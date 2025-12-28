// specificworker.cpp
//
// Este fichero implementa el “cerebro” del componente: inicializa la UI (dos viewers),
// lee el LiDAR, extrae geometría (esquinas/lineas), hace matching contra una sala nominal
// y ejecuta una máquina de estados para ir al centro, buscar puerta, orientarse y cruzarla.
//
// Importante: el código usa *paths relativos* para recursos (por ejemplo `etc/config`),
// por lo que el working directory al ejecutar debe ser la raíz del componente.

#include "specificworker.h"

#include <iostream>

#include <QLoggingCategory>
#include <QtMath>
#include <qcolor.h>

#include <cppitertools/groupby.hpp>

#include <algorithm>
#include <optional>
#include <cmath>
#include <limits>

// =======================
// CONSTRUCTOR / DESTRUCTOR
// =======================

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check)
    : GenericWorker(configLoader, tprx)
{
    // Si `startup_check` está activo, RoboComp lanza el componente en modo “comprobación”
    // (no arranca el bucle normal; sirve para testear que carga config, proxies, etc.).
    this->startup_check_flag = startup_check;

    if (this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
#ifdef HIBERNATION_ENABLED
        // Si tu proyecto soporta hibernación, se arranca el checker.
        hibernationChecker.start(500);
#endif
        // Máquina de estados Qt: se ejecuta en paralelo al compute() normal.
        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        // Si Qt reporta error al arrancar la state machine, lo elevamos como excepción.
        auto error = statemachine.errorString();
        if (error.length() > 0)
        {
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

// =======================
// INITIALIZE
// =======================
// Se llama una vez al arrancar: prepara viewers, dibuja elementos iniciales,
// inicializa la pose (transformación) y deja al robot parado.

void SpecificWorker::initialize()
{
    qInfo() << "Initialize worker";

    if (startup_check_flag)
    {
        startup_check();
        return;
    }

    // =========================
    // 1) VIEWER IZQUIERDO (sensor/robot)
    // =========================
    // Aquí se dibuja el LiDAR, el robot “en su propio frame” y elementos de debug.
    viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);

    if (auto lay = frame->layout())
        lay->addWidget(viewer);
    else
        viewer->setGeometry(frame->rect());

    viewer->scene.setSceneRect(params.GRID_MAX_DIM);
    viewer->fitInView(params.GRID_MAX_DIM, Qt::KeepAspectRatio);

    // Robot dibujado en el viewer izquierdo (debug / referencia).
    {
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;
    }

    // =========================
    // 2) VIEWER DERECHO (mapa / sala nominal)
    // =========================
    // Este viewer se usa para representar la sala nominal (rectángulo) y el robot
    // con la pose estimada sobre ese “mapa”.
    viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);

    if (auto lay = frame_room->layout())
        lay->addWidget(viewer_room);
    else
        viewer_room->setGeometry(frame_room->rect());

    viewer_room->scene.setSceneRect(params.GRID_MAX_DIM);
    viewer_room->fitInView(params.GRID_MAX_DIM, Qt::KeepAspectRatio);

    // De momento se dibuja la sala nominal 0 como rectángulo (mapa base).
    // (En Task 4 normalmente irías cambiando esto según la sala actual).

    // Robot dibujado sobre el viewer_room (pose estimada en el “mapa”).
    {
        auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_room_draw = rr;
    }

    // Muestra la interfaz (frames de Qt ya existentes).
    show();

    // =========================
    // 3) Pose inicial del robot
    // =========================
    // `robot_pose` (Eigen) suele ser una isometría/transform 2D: rotación + traslación.
    // Aquí se inicializa como identidad en el origen.
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2d(0.0, 0.0));


    // =========================
    // Detectar sala inicial (como en el código antiguo)
    // =========================
    {
        auto init_points = read_data();
        if (!init_points.empty())
        {
            const auto &[init_corners, init_lines] =
                room_detector.compute_corners(init_points, &viewer->scene);

            auto res = find_best_room(init_corners, robot_pose);
            if (res.room_idx >= 0)
            {
                current_room_idx = res.room_idx;
                qInfo() << "INITIAL ROOM DETECTED:" << current_room_idx;
            }
            else
                current_room_idx = 0;
        }
        else
            current_room_idx = 0;

        update_room_rect();  // <-- dibuja y hace fitInView a la sala actual
    }


    // =========================
    // 4) TimeSeriesPlotter (gráfica del error)
    // =========================
    // Grafica en el tiempo el error máximo de matching (útil para ver estabilidad).
    TimeSeriesPlotter::Config plotConfig;
    plotConfig.title = "Maximum Match Error Over Time";
    plotConfig.yAxisLabel = "Error (mm)";
    plotConfig.timeWindowSeconds = 15.0;
    plotConfig.autoScaleY = false;
    plotConfig.yMin = 0;
    plotConfig.yMax = 1000;

    time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
    match_error_graph = time_series_plotter->addGraph("", Qt::blue);

    // =========================
    // 5) Parar el robot
    // =========================
    // Dejamos velocidad a cero: adv=0, rot=0.
    move_robot(0.f, 0.f, 0.f);
}

// =======================
// COMPUTE (bucle principal)
// =======================
// Flujo general:
//  1) Leer LiDAR y filtrar puntos
//  2) Detectar esquinas/lineas y estimar centro
//  3) Transformar esquinas nominales al frame del robot y hacer matching (Hungarian)
//  4) Medir error máximo de matching y decidir si estamos “localizados”
//  5) Si estamos localizados, actualizar pose (stub)
//  6) Ejecutar máquina de estados (centro -> puerta -> orientar -> cruzar)
//  7) Enviar velocidad al robot
//  8) Pintar robot en el mapa (viewer derecho)
//  9) Actualizar gráfica y timestamps

void SpecificWorker::compute()
{
    // 1) Lidar + filtros (incluye filtro de puerta para no “ver” puntos fuera de la sala)
    RoboCompLidar3D::TPoints data = read_data();
    if (data.empty())
        return;

    // 2) Esquinas + centro
    // `room_detector.compute_corners()` devuelve esquinas y líneas detectadas.
    const auto& [corners, lines] = room_detector.compute_corners(data, &viewer->scene);

    // Centro estimado a partir de paredes (si el detector puede).
    std::optional<Eigen::Vector2d> center_opt;
    center_opt = room_detector.estimate_center_from_walls(lines);

    // Dibuja puntos LiDAR + centro estimado en el viewer izquierdo.
    draw_lidar(data, center_opt, &viewer->scene);

    // 3) Matching nominal vs medido
    // Transformamos las esquinas nominales de la sala 0 al frame robot actual
    // (robot_pose.inverse(): mundo->robot, según la convención del proyecto).

    const auto nominal_in_robot =
        nominal_rooms[current_room_idx].transform_corners_to(robot_pose.inverse());

    const Match match = hungarian.match(corners, nominal_in_robot);


    // 4) Error máximo entre los matches (peor caso) -> muy útil como indicador de calidad.
    float max_match_error = 99999.f;
    if (not match.empty())
    {
        auto max_it = std::max_element(
            match.begin(), match.end(),
            [](const auto& a, const auto& b) { return std::get<2>(a) < std::get<2>(b); });

        max_match_error = static_cast<float>(std::get<2>(*max_it));

        // DEBUG: imprimir tamaño del match y error
        qInfo() << "[MATCH]"
                << "room:" << current_room_idx
                << "match.size:" << static_cast<int>(match.size())
                << "max_match_error:" << max_match_error;

        if (!match.empty())
        {
            const float e = static_cast<float>(std::get<2>(match.front()));
            qInfo() << "[MATCH-ONE] e:" << e << " sqrt(e):" << std::sqrt(e);
        }



        // Se añade un punto al plot (error vs tiempo) para debug/estabilidad.
        if (time_series_plotter)
            time_series_plotter->addDataPoint(match_error_graph, max_match_error);
    }

    // 5) Heurística simple de “localizado”:
    //    - al menos 3 correspondencias
    //    - error máximo por debajo de un umbral
    localised = (match.size() >= 3 && max_match_error < params.RELOCAL_DONE_MATCH_MAX_ERROR);

    // 6) Si estamos localizados, actualiza la pose (stub por ahora)
    if (localised)
        update_robot_pose(corners, match);

    // 7) Máquina de estados: decide el siguiente estado y los comandos (adv, rot)
    auto [st, adv, rot] = process_state(data, corners, lines, match, viewer);
    state = st;

    // 8) Enviar comandos al robot (adv=avance, rot=rotación)
    move_robot(adv, rot, max_match_error);

    update_debug_panel(adv, rot);

    // 9) Dibujar robot en el viewer_room (mapa) con la pose estimada
    robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());

    // Convertimos la rotación de la matriz a ángulo (rad) y lo pasamos a grados para Qt.
    const double angle = std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0));
    robot_room_draw->setRotation(qRadiansToDegrees(angle));

    // 10) Actualizar GUI (plot) y timestamp
    if (time_series_plotter)
        time_series_plotter->update();

    last_time = std::chrono::high_resolution_clock::now();
}

// =======================
// LECTURA LIDAR + FILTROS
// =======================
// Lee puntos 2D (threshold) del LiDAR y aplica una cadena de filtros:
//  1) quedarnos con el punto más cercano por cada phi (evitar duplicados)
//  2) filtro de aislados (aquí todavía stub)
//  3) filtro de puertas (para no incorporar puntos fuera de la sala actual)

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
    RoboCompLidar3D::TPoints points;

    try
    {
        auto data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH, 12000, 1);
        points = data.points;
    }
    catch (const Ice::Exception& e)
    {
        qWarning() << "[Lidar3D] proxy error:" << e.what();
        return {};
    }

    // 1) Min distancia por phi (equivalente a su filter_min_distance)
    points = filter_same_phi(points);

    // 2) Filtro de aislados (de momento no hace nada, pero deja el hook)
    points = filter_isolated_points(points, 200.f);

    // 3) Filtrar por puertas (evitar puntos de otras salas al mirar por la puerta)
    points = door_detector.filter_points(points, &viewer->scene);

    return points;
}


SpecificWorker::RoomMatchResult
SpecificWorker::find_best_room(const Corners &corners,
                               const Eigen::Affine2d &robot_pose)
{
    RoomMatchResult res;
    if (corners.empty())
        return res;

    for (int i = 0; i < (int)nominal_rooms.size(); ++i)
    {
        const auto nominal_in_robot =
            nominal_rooms[i].transform_corners_to(robot_pose.inverse());

        Match m = hungarian.match(corners, nominal_in_robot);
        if (m.empty()) continue;

        float max_err = 0.f;
        for (const auto &triple : m)
            max_err = std::max(max_err, (float)std::get<2>(triple));

        if (res.room_idx == -1 ||
            m.size() > res.match.size() ||
            (m.size() == res.match.size() && max_err < res.max_error))
        {
            res.room_idx  = i;
            res.match     = std::move(m);
            res.max_error = max_err;
        }
    }
    return res;
}


void SpecificWorker::update_room_rect()
{
    if (!viewer_room) return;
    if (current_room_idx < 0 || current_room_idx >= (int)nominal_rooms.size()) return;

    auto &scene = viewer_room->scene;

    // Guardar transform actual para NO cambiar zoom
    const QTransform current_tf = viewer_room->transform();

    if (room_rect_draw)
    {
        scene.removeItem(room_rect_draw);
        delete room_rect_draw;
        room_rect_draw = nullptr;
    }

    room_rect_draw = scene.addRect(
        nominal_rooms[current_room_idx].rect(),
        QPen(Qt::black, 30)
    );

    if (!room_view_fitted_once)
    {
        // Ajustar SOLO la primera vez (arranque)
        viewer_room->fitInView(room_rect_draw->boundingRect(), Qt::KeepAspectRatio);
        room_view_fitted_once = true;
    }
    else
    {
        // Mantener el zoom actual y solo re-centrar
        viewer_room->setTransform(current_tf);
        viewer_room->centerOn(room_rect_draw);
    }
}


// Agrupa puntos por phi (redondeado) y conserva el más cercano (mínimo r).
RoboCompLidar3D::TPoints SpecificWorker::filter_same_phi(const RoboCompLidar3D::TPoints& points)
{
    if (points.empty())
        return {};

    RoboCompLidar3D::TPoints filtered;

    // groupby de itertools: agrupa consecutivos con misma clave.
    // OJO: si `points` no viene ordenado por phi, esto agrupa “por tramos”.
    for (auto&& [angle, pts] : iter::groupby(points, [](const auto& p)
         {
             float multiplier = std::pow(10.f, 2);     // redondeo a 0.01 rad
             return std::floor(p.phi * multiplier) / multiplier;
         }))
    {
        auto min_it = std::min_element(pts.begin(), pts.end(),
                                       [](const auto& a, const auto& b) { return a.r < b.r; });

        if (min_it != pts.end())
            filtered.emplace_back(*min_it);
    }

    return filtered;
}

// Filtro de aislados:
//  - típico: eliminar puntos que no tengan vecinos a distancia < d
//  - aquí está minimizado a “no filtra nada” por simplicidad.
RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints& points, float d)
{
    Q_UNUSED(d);
    return points;
}

// =======================
// DIBUJO LIDAR (viewer izquierdo)
// =======================
// Dibuja:
//  - puntos LiDAR como rectángulos verdes
//  - centro estimado como círculo rojo (si existe)
//
// Mantiene una lista estática de items para borrar el frame anterior.

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints& points,
                               std::optional<Eigen::Vector2d> center,
                               QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> draw_points;

    if (!scene)
        return;

    // Limpiar items anteriores
    for (auto& p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("LightGreen");
    const QPen pen(color, 10);

    // Dibujar puntos LiDAR
    for (const auto& p : points)
    {
        auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp);
    }

    // Dibujar centro estimado si existe
    if (center.has_value())
    {
        QPen center_pen(Qt::red);
        center_pen.setWidth(15);

        auto c = scene->addEllipse(-50, -50, 100, 100, center_pen);
        c->setPos(center->x(), center->y());
        draw_points.push_back(c);
    }
}

// =======================
// STATE MACHINE (selector de estados)
// =======================
// Según `state` actual, delega a la función que calcula la acción.
// Cada estado devuelve: {next_state, adv, rot}.

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints& data,
                                                     const Corners& corners,
                                                     const Lines& lines,
                                                     const Match& match,
                                                     AbstractGraphicViewer* viewer)
{
    Q_UNUSED(match);
    Q_UNUSED(viewer);

    switch (state)
    {
        case STATE::LOCALISE:
        case STATE::GOTO_ROOM_CENTER:
            return goto_room_center(data, lines);

        case STATE::GOTO_DOOR:
            return goto_door(data);

        case STATE::ORIENT_TO_DOOR:
            return orient_to_door(data);

        case STATE::CROSS_DOOR:
            return cross_door(data);

        case STATE::TURN:
            // TURN aquí es residual (en tu pipeline principal casi no se usa)
            return turn(corners);

        case STATE::IDLE:
            return {STATE::IDLE, 0.f, 0.f};

        default:
            // Fallback “seguro”
            return {STATE::LOCALISE, 0.f, 0.f};
    }
}

// =======================
// ESTADOS (acciones)
// =======================

// LOCALISE: en este snippet no se usa “de verdad”, se deja stub.
SpecificWorker::RetVal SpecificWorker::localise(const Match& match)
{
    Q_UNUSED(match);
    return {STATE::LOCALISE, 0.f, 0.f};
}

// GOTO_ROOM_CENTER:
//  - intenta estimar el centro con líneas (paredes)
//  - si no hay centro: gira para “barrer” la geometría
//  - si hay centro: se aproxima con control proporcional + freno por ángulo
SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints& points,
                                                        const Lines& lines)
{
    Q_UNUSED(points);

    // 1) Estimar centro de la habitación a partir de paredes
    auto center_opt = room_detector.estimate_center_from_walls(lines);

    // 2) NO HAY CENTRO -> girar sobre sí mismo, sin avanzar
    if (!center_opt.has_value())
    {
        float adv = 0.f;       // no avanzar
        float rot = 0.35f;     // girar constante para “encontrar” estructura
        qInfo() << "GOTO_ROOM_CENTER: no center, rotating to find geometry";
        return {STATE::GOTO_ROOM_CENTER, adv, rot};
    }

    // 3) Convertir centro (en frame del robot para este flujo) a variables simples
    Eigen::Vector2d c_world = center_opt.value();
    float cx = static_cast<float>(c_world.x());   // lateral
    float cy = static_cast<float>(c_world.y());   // hacia delante

    // 4) Dibujar el centro en el viewer (debug)
    static QGraphicsEllipseItem* item = nullptr;
    if (item != nullptr)
    {
        viewer->scene.removeItem(item);
        delete item;
    }

    item = viewer->scene.addEllipse(-100, -100, 200, 200,
                                    QPen(Qt::red, 3),
                                    QBrush(Qt::red, Qt::SolidPattern));
    item->setPos(c_world.x(), c_world.y());

    // 5) Distancia al centro
    float dist = std::sqrt(cx * cx + cy * cy);


    // 6. Si ya está en el centro → snap al centro nominal y pasar al siguiente estado

    if (dist < params.RELOCAL_CENTER_EPS)
    {
        relocal_centered = true;

        // Centro geométrico de la sala nominal
        const QRectF r = nominal_rooms[current_room_idx].rect().normalized();
        const Eigen::Vector2d nominal_center(r.center().x(), r.center().y());

        // SNAP: fuerza la pose al centro nominal
        robot_pose.translation() = nominal_center;

        // (Opcional) si quieres dejar el robot "recto" al centrar:
        // robot_pose.linear() = Eigen::Rotation2Dd(0.0).toRotationMatrix();

        qInfo() << "GOTO_ROOM_CENTER: reached -> SNAP to nominal center -> GOTO_DOOR";

        // 🔥 IMPORTANTE: NO ir a TURN, sino a GOTO_DOOR
        return {STATE::GOTO_DOOR, 0.f, 0.f};
    }



    // 7) Ángulo hacia el centro.
    // Convención (en tu código): atan2(x_lateral, y_forward)
    float angle = std::atan2(cx, cy);

    // 8) Control proporcional de giro
    float k_rot = 0.5f;
    float vrot = k_rot * angle;

    // 9) Freno gaussiano: si el ángulo es grande, reduce avance para no “hacer drift”.
    float brake = std::exp(-(angle * angle) / (static_cast<float>(M_PI) / 10.f));
    float adv = params.RELOCAL_MAX_ADV * brake;

    qInfo() << "CENTER:" << cx << cy << "dist:" << dist << "adv:" << adv << "vrot:" << vrot;
    return {STATE::GOTO_ROOM_CENTER, adv, vrot};
}

// GOTO_DOOR:
//  - detecta puertas
//  - elige puerta según `next_use_left` (alternar izq/der)
//  - genera un “punto antes de la puerta” (pre-door) para colocarse bien
//  - se aproxima con robot_controller() (control suave)
SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints& points)
{
    Q_UNUSED(points);

    Doors doors = door_detector.doors();

    if (doors.empty())
    {
        // No hay puertas visibles -> girar para buscar
        float adv = 0.f;
        float rot = 0.3f;
        qInfo() << "GOTO_DOOR: no doors -> rotating to search";
        return {STATE::GOTO_DOOR, adv, rot};
    }

    // 1) Elegir puerta por lado (izquierda/derecha) según next_use_left
    int chosen = -1;
    float best_score = std::numeric_limits<float>::infinity();

    // Intentar primero puertas en el lado deseado
    for (int i = 0; i < static_cast<int>(doors.size()); ++i)
    {
        Eigen::Vector2f c = doors[i].center();
        float angle = std::atan2(c.x(), c.y());   // x lateral, y hacia delante

        // Si quiero izquierda, angle > 0 (según tu convención: positivo = izquierda)
        if (next_use_left && angle <= 0.f) continue;
        if (!next_use_left && angle >= 0.f) continue;

        float score = std::abs(angle);
        if (score < best_score)
        {
            best_score = score;
            chosen = i;
        }
    }

    // Si no hay puertas del lado deseado, coger la más alineada con el frente
    if (chosen == -1)
    {
        best_score = std::numeric_limits<float>::infinity();
        for (int i = 0; i < static_cast<int>(doors.size()); ++i)
        {
            Eigen::Vector2f c = doors[i].center();
            float angle = std::atan2(c.x(), c.y());
            float score = std::abs(angle);

            if (score < best_score)
            {
                best_score = score;
                chosen = i;
            }
        }
    }

    Door& target_door = doors[chosen];

    // 2) Punto "antes de la puerta" en coordenadas robot
    // El 500.f suele ser “distancia hacia atrás” respecto al centro de la puerta.
    Eigen::Vector2f target = target_door.center_before(Eigen::Vector2d::Zero(), 500.f);
    float dist = target.norm();

    // Dibujar target en viewer (debug)
    {
        static QGraphicsItem* door_target_draw = nullptr;
        if (door_target_draw != nullptr)
            viewer->scene.removeItem(door_target_draw);

        door_target_draw = viewer->scene.addEllipse(
            -50, -50, 100, 100,
            QPen(Qt::magenta), QBrush(Qt::magenta));

        door_target_draw->setPos(target.x(), target.y());
    }

    // 3) Si estamos suficientemente cerca del punto previo -> ORIENT_TO_DOOR
    if (dist < params.DOOR_REACHED_DIST)
    {
        qInfo() << "GOTO_DOOR: pre-door point reached, dist:" << dist << " -> ORIENT_TO_DOOR";
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
    }

    // 4) Aproximación suave (arco) con controlador
    auto [adv, rot] = robot_controller(target);

    qInfo() << "GOTO_DOOR: chosen door idx:" << chosen
            << " target:" << target.x() << target.y()
            << " dist:" << dist
            << " adv:" << adv
            << " rot:" << rot;

    return {STATE::GOTO_DOOR, adv, rot};
}

// ORIENT_TO_DOOR:
//  - busca la puerta más alineada (menor |ángulo|)
//  - si error angular es pequeño -> CROSS_DOOR
//  - si no, gira hasta alinear
SpecificWorker::RetVal SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints& points)
{
    Q_UNUSED(points);

    Doors doors = door_detector.doors();

    if (doors.empty())
    {
        float adv = 0.f;
        float rot = 0.3f;
        qInfo() << "ORIENT_TO_DOOR: no doors -> rotating to search";
        return {STATE::ORIENT_TO_DOOR, adv, rot};
    }

    // Elegir la puerta más alineada con el heading (mínimo |ángulo|)
    int best_idx = -1;
    float best_err = std::numeric_limits<float>::infinity();
    float best_angle = 0.f;

    for (int i = 0; i < static_cast<int>(doors.size()); ++i)
    {
        Eigen::Vector2f c = doors[i].center();
        float angle = std::atan2(c.x(), c.y());   // x lateral, y hacia delante
        float err = std::abs(angle);

        if (err < best_err)
        {
            best_err = err;
            best_idx = i;
            best_angle = angle;
        }
    }

    Q_UNUSED(best_idx);

    // Umbral de alineación (~6º)
    const float max_oriented_error = 0.10f;

    if (best_err < max_oriented_error)
    {
        qInfo() << "ORIENT_TO_DOOR: door aligned (angle:" << best_angle << ") -> CROSS_DOOR";
        return {STATE::CROSS_DOOR, 0.f, 0.f};
    }

    // Girar hacia la puerta en el sentido del signo del ángulo
    float rot = (best_angle > 0.f ? 1.f : -1.f) * params.RELOCAL_ROT_SPEED / 2.f;
    float adv = 0.f;

    qInfo() << "ORIENT_TO_DOOR: correcting angle:" << best_angle << " rot:" << rot;
    return {STATE::ORIENT_TO_DOOR, adv, rot};
}

// CROSS_DOOR:
//  - estado temporal: avanza recto durante un tiempo fijo (CROSS_DOOR_DURATION)
//  - cuando termina, alterna el lado de la siguiente puerta (next_use_left) y vuelve a buscar centro.

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints& points)
{
    Q_UNUSED(points);

    static bool first_time = true;
    static std::chrono::time_point<std::chrono::high_resolution_clock> start;

    if (first_time)
    {
        first_time = false;
        start = std::chrono::high_resolution_clock::now();

        qInfo() << "CROSS_DOOR: starting, adv:" << params.CROSS_DOOR_SPEED;
        return {STATE::CROSS_DOOR, params.CROSS_DOOR_SPEED, 0.0f};
    }

    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

    if (ms > static_cast<long>(params.CROSS_DOOR_DURATION * 1000.f))
    {
        // Preparar para el próximo cruce
        first_time = true;

        // Alternar lado para la próxima elección de puerta (izq/der)
        next_use_left = !next_use_left;

        // ==========================
        // CAMBIO DE SALA (paso 4)
        // ==========================
        if (!nominal_rooms.empty())
        {
            current_room_idx = (current_room_idx + 1) % nominal_rooms.size();
            qInfo() << "CROSS_DOOR: room changed to" << current_room_idx;

            // (Opcional pero recomendable) reset de pose relativa a la nueva sala
            robot_pose.setIdentity();

            // Reset de flags de relocalización (opcional según tu FSM)
            relocal_centered = false;
            localised        = false;

            // Redibujar rectángulo de la sala y reajustar zoom en el viewer derecho
            update_room_rect();
        }

        qInfo() << "CROSS_DOOR: finished after" << ms << "ms"
                << " -> GOTO_ROOM_CENTER. next_use_left:" << next_use_left;

        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }

    // Seguir cruzando recto
    return {STATE::CROSS_DOOR, params.CROSS_DOOR_SPEED, 0.0f};
}


// TURN (residual):
//  - si no hay puertas: girar buscando
//  - si hay puertas: gira suave (pero en tu pipeline normalmente no llega aquí)
SpecificWorker::RetVal SpecificWorker::turn(const Corners& corners)
{
    Q_UNUSED(corners);

    Doors doors = door_detector.doors();

    if (doors.empty())
    {
        float adv = 0.f;
        float rot = 0.3f;
        qInfo() << "TURN: no doors -> rotating";
        return {STATE::TURN, adv, rot};
    }

    float adv = 0.f;
    float rot = 0.2f;
    qInfo() << "TURN: doors present but TURN unused in pipeline";
    return {STATE::TURN, adv, rot};
}

// =======================
// AUXILIARES VARIOS
// =======================

SpecificWorker::RetVal SpecificWorker::update_pose(const Corners& corners, const Match& match)
{
    Q_UNUSED(corners);
    Q_UNUSED(match);
    return {STATE::UPDATE_POSE, 0.f, 0.f};
}

// Devuelve el índice del punto LiDAR cuya phi esté más cerca del ángulo dado.
std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const auto& points, float angle)
{
    if (points.empty())
        return std::unexpected("No points");

    int best_idx = -1;
    float best_error = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < points.size(); ++i)
    {
        float err = std::abs(points[i].phi - angle);
        if (err < best_error)
        {
            best_error = err;
            best_idx = static_cast<int>(i);
        }
    }

    if (best_idx == -1)
        return std::unexpected("No valid index");

    return best_idx;
}

void SpecificWorker::print_match(const Match& match, float error) const
{
    Q_UNUSED(error);
    Q_UNUSED(match);
    // Si quieres debug, imprime aquí el contenido del match (ids, distancias, etc.).
}

// =======================
// POSE / CONTROL
// =======================

// update_robot_pose():
//  - en Task 2/3 suele aplicar una corrección de pose (delta) calculada con corners+match.
//  - aquí está como stub (no actualiza).
bool SpecificWorker::update_robot_pose(const Corners& corners, const Match& match)
{
    Q_UNUSED(corners);
    Q_UNUSED(match);
    // Aquí más adelante meterás tus ecuaciones de la Task 2 (solve_pose + aplicar incremento)
    return false;
}

// solve_pose():
//  - normalmente resuelve (x, y, theta) que mejor alinea corners detectadas con nominales.
//  - stub: no devuelve corrección.
Eigen::Vector3d SpecificWorker::solve_pose(const Corners& corners, const Match& match)
{
    Q_UNUSED(corners);
    Q_UNUSED(match);
    return Eigen::Vector3d::Zero();
}

// predict_robot_pose():
//  - si tuvieras odometría, aquí podrías predecir pose antes de corregir.
void SpecificWorker::predict_robot_pose()
{
    // Stub: no hacemos predicción todavía
}

// robot_controller():
// Controlador “simple” para ir a un target en el frame robot.
//
// Convención de frame robot usada en TODO el fichero:
//   x = lateral (derecha +, izquierda -)   [según tu comentario original]
//   y = hacia delante
//
// Nota: en tu lógica también usas que ángulo positivo = objetivo a la izquierda
// (eso depende del sistema de ejes; aquí se conserva tal cual tu convención con atan2(lx, fy)).
std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f& target)
{
    const float lx = target.x();     // lateral
    const float fy = target.y();     // forward
    const float dist = std::sqrt(lx * lx + fy * fy);

    if (dist < 1e-3f)
        return {0.f, 0.f};

    // Ángulo hacia el objetivo (según tu convención)
    const float angle = std::atan2(lx, fy);

    // Si estamos muy cerca del objetivo, paramos
    if (dist < params.RELOCAL_CENTER_EPS)
        return {0.f, 0.f};

    // Caso especial: objetivo detrás (fy negativo) -> girar en sitio
    if (fy < 0.f)
    {
        float rot = (angle > 0.f ? 1.f : -1.f) * params.RELOCAL_ROT_SPEED;
        return {0.f, rot};
    }

    // --- Control angular proporcional ---
    const float k_w = 1.0f;
    float rot = k_w * angle;
    rot = std::clamp(rot, -params.RELOCAL_ROT_SPEED, params.RELOCAL_ROT_SPEED);

    // --- Avance penalizado por ángulo ---
    // Si el objetivo está “de lado”, frenamos avance para evitar curvas agresivas.
    const float max_adv = params.RELOCAL_MAX_ADV;
    const float sigma = static_cast<float>(M_PI) / 4.f;  // 45º
    const float angle2 = angle * angle;

    float brake = std::exp(-angle2 / (2.f * sigma * sigma));  // [0,1]
    float adv = max_adv * brake;

    return {adv, rot};
}

// move_robot():
//  - guarda el comando en un buffer (para debug/plot) y lo envía al proxy OmniRobot.
//  - el proxy se llama con (advx, advz, rot). Aquí se usa advz=adv, advx=0.

void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
    Q_UNUSED(max_match_error);

    // ============================
    // 1) ODOMETRÍA SIMPLE (como en tu cpp antiguo)
    // ============================
    auto now = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now - last_time).count() / 1000.f;

    // Evitar dt raros por parones, primer ciclo, etc.
    if (dt < 0.f || dt > 1.f)
        dt = 0.f;

    // Incrementos
    const double d      = static_cast<double>(adv) * dt;   // mm
    const double dtheta = static_cast<double>(rot) * dt;   // rad

    // IMPORTANTE: tu convención en el antiguo era avanzar en eje Y (forward)
    Eigen::Affine2d delta = Eigen::Affine2d::Identity();
    delta.translate(Eigen::Vector2d(0.0, d));
    delta.rotate(dtheta);

    // robot_pose: ROBOT -> SALA NOMINAL
    robot_pose = robot_pose * delta;

    // ============================
    // 2) GUARDAR COMANDO
    // ============================
    long ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()).count();

    std::tuple<float, float, float, long> cmd{0.f, adv, rot, ms};
    commands_buffer.put(std::move(cmd));
    last_velocities = cmd;

    // ============================
    // 3) ENVIAR A LA BASE
    // ============================
    try
    {
        omnirobot_proxy->setSpeedBase(0.f, adv, rot);
    }
    catch (const Ice::Exception &e)
    {
        qWarning() << "[OmniRobot] setSpeedBase error:" << e.what();
    }

    // actualizar tiempo para la próxima odometría
    last_time = now;
}


void SpecificWorker::update_debug_panel(float adv, float rot)
{
    const double x = robot_pose.translation().x();
    const double y = robot_pose.translation().y();

    const double angle_rad = std::atan2(
        robot_pose.linear()(1, 0),
        robot_pose.linear()(0, 0));

    const double angle_deg = qRadiansToDegrees(angle_rad);

    // ==== X, Y, ANGLE ====
    if (auto lcdX = this->findChild<QLCDNumber*>("lcdNumber_x"))
        lcdX->display(x);

    if (auto lcdY = this->findChild<QLCDNumber*>("lcdNumber_y"))
        lcdY->display(y);

    if (auto lcdAngle = this->findChild<QLCDNumber*>("lcdNumber_angle"))
        lcdAngle->display(angle_deg);

    // ==== ADV, ROT ====
    if (auto lcdAdv = this->findChild<QLCDNumber*>("lcdNumber_adv"))
        lcdAdv->display(adv);

    if (auto lcdRot = this->findChild<QLCDNumber*>("lcdNumber_rot"))
        lcdRot->display(rot);

    // ==== Estado de la FSM ====
    if (auto lblState = this->findChild<QLabel*>("label_state"))
        lblState->setText(QString::fromLatin1(to_string(state)));
}

// =======================
// EMERGENCIA / RESTORE / STARTUP
// =======================

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
}

void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompCamera360RGB you can call this methods:
// RoboCompCamera360RGB::TImage this->camera360rgb_proxy->getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage
/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)
/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData
/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()
/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams
