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

    // =========================
    // 1) VIEWER IZQUIERDO
    // =========================
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

    // =========================
    // 2) VIEWER DERECHO (mapa)
    // =========================
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
    {
        auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH,
                                               params.ROBOT_LENGTH,
                                               0, 100, QColor("Blue"));
        robot_room_draw = rr;
    }

    // mostrar UI
    show();

    // =========================
    // 3) Pose inicial del robot
    // =========================
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2d(0.0, 0.0));

    // =========================
    // 4) TimeSeriesPlotter (error de match)
    // =========================
    TimeSeriesPlotter::Config plotConfig;
    plotConfig.title             = "Maximum Match Error Over Time";
    plotConfig.yAxisLabel        = "Error (mm)";
    plotConfig.timeWindowSeconds = 15.0;
    plotConfig.autoScaleY        = false;
    plotConfig.yMin              = 0;
    plotConfig.yMax              = 1000;

    time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
    match_error_graph   = time_series_plotter->addGraph("", Qt::blue);

    // =========================
    // 5) Parar el robot
    // =========================
    move_robot(0.f, 0.f, 0.f);
}

void SpecificWorker::compute()
{
    // 1) Lidar + filtros (incluye filtro de puerta)
    RoboCompLidar3D::TPoints data = read_data();
    if (data.empty())
        return;

    // 2) Esquinas + centro
    const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);
    const auto center_opt        = room_detector.estimate_center_from_walls(lines);

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

    // 5) Localised: nº de matches y error
    localised = (match.size() >= 3 && max_match_error < params.RELOCAL_DONE_MATCH_MAX_ERROR);

    // 6) Actualizar pose si estamos localizados (cuando implementes update_robot_pose)
    if (localised)
        update_robot_pose(corners, match);

    // 7) State machine (por ahora sólo GOTO_ROOM_CENTER)
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



// =======================
// LECTURA LIDAR + FILTROS
// =======================

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
    RoboCompLidar3D::TPoints points;

    try
    {
        // usamos el lidar alto por defecto
        auto data = lidar3d_proxy->getLidarDataWithThreshold2d(
            params.LIDAR_NAME_HIGH, 5000, 1);
        points = data.points;
    }
    catch (const Ice::Exception &e)
    {
        qWarning() << "[Lidar3D] proxy error:" << e.what();
        return {};
    }

    // filtros propios
    points = filter_same_phi(points);
    points = filter_isolated_points(points, 200.f);  // umbral provisional

    // filtro de puerta (elimina puntos fuera de la sala)
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

// =======================
// DIBUJO LIDAR
// =======================

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

// =======================
// COMPUTE
// =======================


// =======================
// STATE MACHINE (stubs)
// =======================

SpecificWorker::RetVal
SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data,
                              const Corners &corners,
                              const Lines   &lines,
                              const Match   &match,
                              AbstractGraphicViewer *viewer)
{
    Q_UNUSED(data);
    Q_UNUSED(corners);
    Q_UNUSED(match);
    Q_UNUSED(viewer);

    // Por ahora, siempre vamos al centro de la habitación
    state = STATE::GOTO_ROOM_CENTER;
    return goto_room_center(data, lines);
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
    return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
}

SpecificWorker::RetVal
SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
    Q_UNUSED(points);
    return {STATE::CROSS_DOOR, 0.f, 0.f};
}

SpecificWorker::RetVal
SpecificWorker::localise(const Match &match)
{
    Q_UNUSED(match);
    return {STATE::LOCALISE, 0.f, 0.f};
}

SpecificWorker::RetVal
SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points, const Lines &lines)
{
    Q_UNUSED(points);

    // Estimamos el centro de la sala a partir de las paredes
    const auto center_opt = room_detector.estimate_center_from_walls(lines);
    if (not center_opt.has_value())
    {
        // No sabemos bien el centro -> no nos movemos
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }

    Eigen::Vector2d center_world = center_opt.value();

    // Vector desde robot a centro en coordenadas mundo
    Eigen::Vector2d diff_world = center_world - robot_pose.translation();

    // Pasar a frame del robot: R^T * (center - pos)
    Eigen::Vector2d diff_robot = robot_pose.linear().transpose() * diff_world;

    Eigen::Vector2f target_robot(static_cast<float>(diff_robot.x()),
                                 static_cast<float>(diff_robot.y()));

    auto [adv, rot] = robot_controller(target_robot);
    return {STATE::GOTO_ROOM_CENTER, adv, rot};
}


SpecificWorker::RetVal
SpecificWorker::update_pose(const Corners &corners, const Match &match)
{
    Q_UNUSED(corners);
    Q_UNUSED(match);
    return {STATE::LOCALISE, 0.f, 0.f};
}

SpecificWorker::RetVal
SpecificWorker::turn(const Corners &corners)
{
    Q_UNUSED(corners);
    return {STATE::TURN, 0.f, 0.f};
}

// =======================
// AUXILIARES VARIOS
// =======================

std::expected<int, std::string>
SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
    if (points.empty())
        return std::unexpected("No points");

    int   best_idx   = -1;
    float best_error = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < points.size(); ++i)
    {
        float err = std::abs(points[i].phi - angle);
        if (err < best_error)
        {
            best_error = err;
            best_idx   = static_cast<int>(i);
        }
    }

    if (best_idx == -1)
        return std::unexpected("No valid index");

    return best_idx;
}

void SpecificWorker::print_match(const Match &match, float error) const
{
    Q_UNUSED(error);
    Q_UNUSED(match);
    // Si quieres debug, imprime aquí el contenido del match.
}

// =======================
// POSE / CONTROL
// =======================

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
    // target está en el frame del ROBOT: x hacia delante, y hacia la izquierda
    const float tx = target.x();
    const float ty = target.y();

    const float dist  = std::sqrt(tx*tx + ty*ty);
    const float angle = std::atan2(ty, tx);   // ángulo hacia el objetivo en el frame del robot

    // Si ya estamos suficientemente cerca del centro, paramos
    if (dist < params.RELOCAL_CENTER_EPS)
        return {0.f, 0.f};

    // Ganancias (puedes ajustarlas si va muy brusco)
    const float k_v = params.RELOCAL_KP;      // pasa distancia (mm) a velocidad
    const float k_w = 1.0f;                   // ganancia para rotación

    float adv = k_v * dist;      // mm/s
    float rot = k_w * angle;     // rad/s

    // Reducir avance si el ángulo es grande (no avances de espaldas)
    adv *= std::cos(angle);
    if (adv < 0.f)
        adv = 0.f;

    // Saturaciones
    adv = std::clamp(adv, -params.RELOCAL_MAX_ADV,  params.RELOCAL_MAX_ADV);
    rot = std::clamp(rot, -params.RELOCAL_ROT_SPEED, params.RELOCAL_ROT_SPEED);

    return {adv, rot};
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
