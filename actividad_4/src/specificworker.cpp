#include "specificworker.h"

#include <iostream>
#include <qcolor.h>
#include <QLoggingCategory>
#include <QtMath>
#include <cppitertools/groupby.hpp>

#include <algorithm>
#include <optional>
#include <cmath>
#include <limits>

// =======================
// CONSTRUCTOR / DESTRUCTOR
// =======================

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
    this->startup_check_flag = startup_check;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
#ifdef HIBERNATION_ENABLED
        hibernationChecker.start(500);
#endif

        // Example statemachine:
        /***
        //Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
        states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period,
                                                            std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
                                                            std::bind(&SpecificWorker::customEnter, this), // On-enter function
                                                            std::bind(&SpecificWorker::customExit, this)); // On-exit function

        //Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
        states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
        states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

        //Add your custom state
        statemachine.addState(states["CustomState"].get());
        ***/

        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        auto error = statemachine.errorString();
        if (error.length() > 0){
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
    qInfo() << "PRUEBA";
    std::cout << "Initialize worker" << std::endl;

    if (startup_check_flag)
    {
        startup_check();
        return;
    }

    viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
    if (auto lay = frame->layout())
        lay->addWidget(viewer);
    else
        viewer->setGeometry(frame->rect());

    viewer->scene.setSceneRect(params.GRID_MAX_DIM);
    viewer->fitInView(params.GRID_MAX_DIM, Qt::KeepAspectRatio);

    // robot en el viewer izquierdo
    {
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH,
                                        params.ROBOT_LENGTH,
                                        0, 100, QColor("Blue"));
        robot_draw = r;
    }

    // 2) VIEWER DERECHO (mapa)
    viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
    if (auto lay = frame_room->layout())
        lay->addWidget(viewer_room);
    else
        viewer_room->setGeometry(frame_room->rect());

    viewer_room->scene.setSceneRect(params.GRID_MAX_DIM);
    viewer_room->fitInView(params.GRID_MAX_DIM, Qt::KeepAspectRatio);

    // dibujar sala nominal 0
    viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));

    // robot en viewer_room
    auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH,
                                               params.ROBOT_LENGTH,
                                               0, 100, QColor("Blue"));
    robot_room_draw = rr;


    // mostrar UI
    show();

    // 3) Pose inicial del robot
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2d(0.0, 0.0));

    // 4) TimeSeriesPlotter
    TimeSeriesPlotter::Config plotConfig;
    plotConfig.title             = "Maximum Match Error Over Time";
    plotConfig.yAxisLabel        = "Error (mm)";
    plotConfig.timeWindowSeconds = 15.0;
    plotConfig.autoScaleY        = false;
    plotConfig.yMin              = 0;
    plotConfig.yMax              = 1000;

    time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
    match_error_graph   = time_series_plotter->addGraph("", Qt::blue);

    // 5) Parar el robot
    move_robot(0.f, 0.f, 0.f);
}

void SpecificWorker::compute()
{
    // 1) Lidar + filtros (incluye filtro de puerta)
    RoboCompLidar3D::TPoints data = read_data();
    if (data.empty())
        return;

    // 2) Esquinas
    const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);

    // Centro SOLO se usa/visualiza en la primera habitación
    std::optional<Eigen::Vector2d> center_opt;
    if (!crossed_door)
        center_opt = room_detector.estimate_center_from_walls(lines);

    draw_lidar(data, center_opt, &viewer->scene);

    // 3) Matching nominal vs medido (nominal en frame ROBOT)
    const auto nominal_in_robot =
        nominal_rooms[0].transform_corners_to(robot_pose.inverse());

    const Match match = hungarian.match(corners, nominal_in_robot);

    // 4) Max match error
    float max_match_error = 99999.f;
    if (not match.empty())
    {
        auto max_it = std::max_element(
            match.begin(), match.end(),
            [](const auto &a, const auto &b)
            { return std::get<2>(a) < std::get<2>(b); });

        max_match_error = static_cast<float>(std::get<2>(*max_it));
        if (time_series_plotter)
            time_series_plotter->addDataPoint(match_error_graph, max_match_error);
    }

    // 5) Localised
    localised = (match.size() >= 3 && max_match_error < params.RELOCAL_DONE_MATCH_MAX_ERROR);

    // 6) Actualizar pose si estamos localizados (cuando implementes update_robot_pose)
    if (localised)
        update_robot_pose(corners, match);

    // 7) Máquina de estados
    auto [st, adv, rot] = process_state(data, corners, lines, match, viewer);
    state = st;

    // 8) Enviar comandos al robot
    move_robot(adv, rot, max_match_error);

    // 9) Dibujar robot en viewer_room con la pose estimada
    robot_room_draw->setPos(robot_pose.translation().x(),
                            robot_pose.translation().y());

    const double angle = std::atan2(robot_pose.linear()(1, 0),
                                    robot_pose.linear()(0, 0));
    robot_room_draw->setRotation(qRadiansToDegrees(angle));

    // 10) Actualizar GUI
    if (time_series_plotter)
        time_series_plotter->update();

    last_time = std::chrono::high_resolution_clock::now();
}

// LECTURA LIDAR + FILTROS
RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
    RoboCompLidar3D::TPoints points;

    try
    {
        auto data = lidar3d_proxy->getLidarDataWithThreshold2d(
            params.LIDAR_NAME_HIGH, 12000, 1);
        points = data.points;
    }
    catch (const Ice::Exception &e)
    {
        qWarning() << "[Lidar3D] proxy error:" << e.what();
        return {};
    }

    // 1) Min distancia por phi (equivalente a filter_min_distance de ellos)
    points = filter_same_phi(points);

    // 2) De momento, no hacemos filtro de aislados agresivo
    points = filter_isolated_points(points, 200.f);

   // 3) MUY IMPORTANTE: filtrar por puerta
    points = door_detector.filter_points(points, &viewer->scene);

    return points;
}

RoboCompLidar3D::TPoints
SpecificWorker::filter_same_phi(const RoboCompLidar3D::TPoints &points)
{
    if (points.empty())
        return {};

    RoboCompLidar3D::TPoints filtered;
    for (auto &&[angle, pts] : iter::groupby(points, [](const auto &p)
    {
        float multiplier = std::pow(10.f, 2);   // redondeo a 0.01 rad
        return std::floor(p.phi * multiplier) / multiplier;
    }))
    {
        auto min_it = std::min_element(pts.begin(), pts.end(),
                                       [](const auto &a, const auto &b)
                                       { return a.r < b.r; });
        if (min_it != pts.end())
            filtered.emplace_back(*min_it);
    }
    return filtered;
}

RoboCompLidar3D::TPoints
SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d)
{
    // versión mínima: no filtra nada todavía
    Q_UNUSED(d);
    return points;
}

// DIBUJO LIDAR
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points,
                                std::optional<Eigen::Vector2d> center,
                                QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem *> draw_points;
    if (!scene)
        return;

    // limpiar anteriores
    for (auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("LightGreen");
    const QPen   pen(color, 10);

    for (const auto &p : points)
    {
        auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp);
    }

    // dibujar centro estimado si lo hay
    if (center.has_value())
    {
        QPen center_pen(Qt::red);
        center_pen.setWidth(15);
        auto c = scene->addEllipse(-50, -50, 100, 100, center_pen);
        c->setPos(center->x(), center->y());
        draw_points.push_back(c);
    }
}

// STATE MACHINE (stubs)
SpecificWorker::RetVal
SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data,
                              const Corners &corners,
                              const Lines   &lines,
                              const Match   &match,
                              AbstractGraphicViewer *viewer)
{
    Q_UNUSED(corners);
    Q_UNUSED(match);
    Q_UNUSED(viewer);

    switch (state)
    {
    case STATE::LOCALISE:
    case STATE::GOTO_ROOM_CENTER:
        return goto_room_center(data, lines);

    case STATE::TURN:
        return turn(corners);

    case STATE::CROSS_DOOR:
        return cross_door(data);

    case STATE::IDLE:
        return {STATE::IDLE, 0.f, 0.f};

    default:
        return {STATE::LOCALISE, 0.f, 0.f};
    }
}


SpecificWorker::RetVal
SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points)
{
    Q_UNUSED(points);
    return {STATE::GOTO_DOOR, 0.f, 0.f};
}

SpecificWorker::RetVal
SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points)
{
    Q_UNUSED(points);
    return {STATE::IDLE, 0.f, 0.f};
}

SpecificWorker::RetVal
SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
    Q_UNUSED(points);

    // Inicializar al entrar por primera vez
    if (!crossing_door)
    {
        crossing_door    = true;
        cross_door_start = std::chrono::high_resolution_clock::now();
        qInfo() << "CROSS_DOOR: starting straight crossing. target travel (mm):"
                << door_travel_target_mm;
    }

    auto now = std::chrono::high_resolution_clock::now();
    float elapsed_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - cross_door_start).count() / 1000.f;

    // Distancia recorrida aproximada = velocidad * tiempo
    float travelled_mm = params.CROSS_DOOR_SPEED * elapsed_sec;

    // Mientras no hayamos llegado a la distancia objetivo, avanzamos recto
    if (travelled_mm < door_travel_target_mm)
    {
        float adv = params.CROSS_DOOR_SPEED;  // mm/s hacia delante
        float rot = 0.f;
        return {STATE::CROSS_DOOR, adv, rot};
    }

    // Hemos recorrido suficiente: damos por cruzada la puerta
    crossing_door = false;
    crossed_door  = true;
    red_patch_detected = false;   // por si quieres buscar otra puerta en el futuro

    qInfo() << "CROSS_DOOR: finished crossing. travelled (mm):" << travelled_mm
            << " -> switching to GOTO_ROOM_CENTER (SECOND room)";

    return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
}


SpecificWorker::RetVal
SpecificWorker::localise(const Match &match)
{
    Q_UNUSED(match);
    return {STATE::LOCALISE, 0.f, 0.f};
}

SpecificWorker::RetVal
SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points,
                                 const Lines &lines)
{
    Q_UNUSED(points);

    // 1. Estimar centro de la habitación a partir de las paredes
    auto center_opt = room_detector.estimate_center_from_walls(lines);

    // 2. NO HAY CENTRO → girar sobre sí mismo, sin avanzar
    if (!center_opt.has_value())
    {
        float adv = 0.f;       // NO avanzar
        float rot = 0.35f;     // Giro constante para “barrer” la sala
        qInfo() << "GOTO_ROOM_CENTER: no center, rotating to find geometry";
        return {STATE::GOTO_ROOM_CENTER, adv, rot};
    }

    // 3. Convertir centro a coordenadas del robot
    Eigen::Vector2d c_world = center_opt.value();
    float cx = static_cast<float>(c_world.x());   // lateral
    float cy = static_cast<float>(c_world.y());   // hacia delante

    // 4. Dibujar el centro en el viewer (debug)
    static QGraphicsEllipseItem *item = nullptr;
    if (item != nullptr)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    item = viewer->scene.addEllipse(-100, -100, 200, 200,
                                    QPen(Qt::red, 3),
                                    QBrush(Qt::red, Qt::SolidPattern));
    item->setPos(c_world.x(), c_world.y());

    // 5. Distancia al centro
    float dist = std::sqrt(cx * cx + cy * cy);

    // 6. Si ya está en el centro → pasar a TURN SIEMPRE
    if (dist < params.RELOCAL_CENTER_EPS)
    {
        relocal_centered = true;
        qInfo() << "GOTO_ROOM_CENTER: center reached -> TURN";
        return {STATE::TURN, 0.f, 0.f};
    }

    // 7. Ángulo hacia el centro
    float angle = std::atan2(cx, cy);

    // 8. Control proporcional de giro
    float k_rot = 0.5f;
    float vrot  = k_rot * angle;

    // 9. Freno gaussiano
    float brake = std::exp(-(angle * angle) / (static_cast<float>(M_PI) / 10.f));
    float adv   = params.RELOCAL_MAX_ADV * brake;

    qInfo() << "CENTER:" << cx << cy
            << "dist:" << dist
            << "adv:"  << adv
            << "vrot:" << vrot;

    return {STATE::GOTO_ROOM_CENTER, adv, vrot};
}

SpecificWorker::RetVal
SpecificWorker::turn(const Corners &corners)
{
    Q_UNUSED(corners);

    // FASE 1: BUSCAR PARCHE ROJO
    if (!red_patch_detected)
    {
        const auto [seen, spin] = rc::ImageProcessor::check_colour_patch_in_image(
                                      camera360rgb_proxy,
                                      QColor("red"),
                                      nullptr,
                                      1500);      // umbral píxeles rojos

        Q_UNUSED(spin);

        if (!seen)
        {
            // Todavía no vemos el parche rojo -> girar SIEMPRE en el mismo sentido
            float adv = 0.f;
            float rot = 0.3f;   // siempre mismo signo

            return {STATE::TURN, adv, rot};
        }

        // Aquí hemos visto el parche rojo centrado
        red_patch_detected = true;
        qInfo() << "TURN: RED PATCH DETECTED -> start looking for DOOR with LIDAR";
        // seguimos en TURN pero entramos en la FASE 2 (LIDAR)
    }

    // FASE 2: GIRAR HASTA TENER LA PUERTA DELANTE
    auto door_exp = door_detector.get_current_door();
    const float angle_tol = 0.10f;   // ~6º de tolerancia
    float adv = 0.f;
    float rot = 0.6f;                // SIEMPRE mismo sentido

    if (door_exp.has_value())
    {
        const Door &d = door_exp.value();
        Eigen::Vector2f c = d.center();            // centro de la puerta en frame robot
        float dist  = c.norm();
        float angle = std::atan2(c.x(), c.y());    // x lateral, y hacia delante

        // Si ya estamos alineados con la puerta -> empezar a cruzarla
        // Si ya estamos alineados con la puerta -> preparar cruce recto
        if (std::abs(angle) < angle_tol)
        {
            // Queremos avanzar dist hasta la puerta + un extra para estar dentro de la segunda sala
            const float extra_mm = 2000.f;  // 2 metros más allá de la puerta (ajusta a ojo)
            door_travel_target_mm = dist + extra_mm;

            crossing_door   = false;   // para que cross_door se inicialice
            crossed_door    = false;   // aún no la hemos cruzado completamente

            qInfo() << "TURN: DOOR ALIGNED. dist:" << dist
                    << " -> target travel (mm):" << door_travel_target_mm
                    << " -> switching to CROSS_DOOR";

            return {STATE::CROSS_DOOR, 0.f, 0.f};
        }

        qInfo() << "TURN: rotating (door seen). angle:" << angle << " dist:" << dist;
    }
    else
    {
        qInfo() << "TURN: rotating (door NOT seen yet)";
    }

    // Mientras no esté alineado: seguir girando siempre igual
    return {STATE::TURN, adv, rot};
}

// AUXILIARES VARIOS
SpecificWorker::RetVal
SpecificWorker::update_pose(const Corners &corners, const Match &match)
{
    Q_UNUSED(corners);
    Q_UNUSED(match);
    return {STATE::UPDATE_POSE, 0.f, 0.f};
}
std::expected<int, std::string>
SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
    int   best_idx   = -1;
    return best_idx;
}

void SpecificWorker::print_match(const Match &match, float error) const
{
    Q_UNUSED(error);
    Q_UNUSED(match);
}

// POSE / CONTROL
bool SpecificWorker::update_robot_pose(const Corners &corners, const Match &match)
{
    Q_UNUSED(corners);
    Q_UNUSED(match);
    // Aquí más adelante meterás tus ecuaciones de la Task 2 (solve_pose + aplicar incremento)
    return false;
}

Eigen::Vector3d SpecificWorker::solve_pose(const Corners &corners, const Match &match)
{
    Q_UNUSED(corners);
    Q_UNUSED(match);
    // Stub: sin corrección
    return Eigen::Vector3d::Zero();
}

void SpecificWorker::predict_robot_pose()
{
    // Stub: no hacemos predicción todavía
}
std::tuple<float, float>
SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
    return {0.f, 0.f};
}

void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
    Q_UNUSED(max_match_error);

    // Guardar en el buffer (por si luego quieres trazar velocidades)
    auto now = std::chrono::high_resolution_clock::now();
    long ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch())
               .count();
    std::tuple<float, float, float, long> cmd{0.f, adv, rot, ms};
    commands_buffer.put(std::move(cmd));
    last_velocities = cmd;

    // Enviar al robot
    try
    {
        omnirobot_proxy->setSpeedBase(0.f, adv, rot);
    }
    catch (const Ice::Exception &e)
    {
        qWarning() << "[OmniRobot] setSpeedBase error:" << e.what();
    }
}

// EMERGENCIA / RESTORE / STARTUP

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