#include "specificworker.h"

#include <iostream>
#include <qcolor.h>
#include <QRect>
#include <QLoggingCategory>
#include <cppitertools/groupby.hpp>
#include <QtMath>                  // Qt5: para qRadiansToDegrees
// #include <sys/socket.h>         // Sólo si lo necesitas

#include "hungarian.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <optional>
#include <chrono>
#include <cmath>

// =====================================================
////////////HOLAAAAAAAAAAAAAAAAA
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check)
    : GenericWorker(configLoader, tprx), startup_check_flag(startup_check)
{
    if (this->startup_check_flag)
        this->startup_check();
    else
    {
#ifdef HIBERNATION_ENABLED
        hibernationChecker.start(500);
#endif
        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        auto error = statemachine.errorString();
        if (!error.isEmpty())
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

void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

    this->dimensions = QRectF(-6000, -3000, 12000, 6000);

    // -------- izquierda (frame) --------
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    if (auto lay = frame->layout()) lay->addWidget(viewer); else viewer->setGeometry(frame->rect());
    viewer->scene.setSceneRect(dimensions);
    viewer->show();
    viewer->fitInView(dimensions, Qt::KeepAspectRatio);

    const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
    robot_polygon = std::get<0>(rob);

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    // -------- derecha (frame_room) con mapa fijo --------
    if (auto fr = this->findChild<QFrame*>("frame_room"))
    {
        viewer_room = new AbstractGraphicViewer(fr, this->dimensions);
        if (auto lay = fr->layout()) lay->addWidget(viewer_room); else viewer_room->setGeometry(fr->rect());
        viewer_room->scene.setSceneRect(dimensions);
        draw_room(&viewer_room->scene, dimensions);
        viewer_room->show();
        viewer_room->fitInView(dimensions, Qt::KeepAspectRatio);

        const auto rob2 = viewer_room->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
        robot_polygon_room = std::get<0>(rob2);
    }
    else
        qWarning() << "[UI] No encuentro 'frame_room' en el .ui";

    // pose para la izquierda (localización)
    robot_pose = Eigen::Isometry2d::Identity();
}

/**
 * Método compute — ejecutado periódicamente según el periodo configurado
 */
void SpecificWorker::compute()
{
    // --- 1) Leer LIDAR y filtrar ---
    RoboCompLidar3D::TPoints points;
    bool lidar_ok = true;
    try
    {
        auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 5000, 1);
        points = data.points;
    }
    catch (const Ice::Exception &e)
    {
        qWarning() << "[Lidar3D] proxy error:" << e.what();
        lidar_ok = false;
    }

    std::optional<RoboCompLidar3D::TPoints> filtered;
    if (lidar_ok)
        filtered = filter_lidar(points);

    // Dibujo LiDAR en la izquierda
    if (viewer && filtered && !filtered->empty())
        draw_lidar(*filtered, &viewer->scene);

    // --- 2) Localización (no afecta al movimiento) ---
    if (lidar_ok && filtered && !filtered->empty())
    {
        try
        {
            // 21.a) extraer esquinas medidas
            Corners measured = room_detector.compute_corners(*filtered, viewer ? &viewer->scene : nullptr);

            if (!measured.empty())
            {
                // 21.c) nominales en frame ROBOT
                Corners nominal_in_robot = room.transform_corners_to(robot_pose.inverse());

                // Hungarian
                std::vector<Eigen::Vector2d> M, N;
                M.reserve(measured.size());  N.reserve(nominal_in_robot.size());
                for (const auto &[p, a, q] : measured)          M.emplace_back(p.x(), p.y());
                for (const auto &[p, a, q] : nominal_in_robot)  N.emplace_back(p.x(), p.y());

                if (!M.empty() && !N.empty())
                {
                    const int m = (int)M.size(), n = (int)N.size();
                    Eigen::MatrixXd C(m, n);
                    for (int i = 0; i < m; ++i)
                        for (int j = 0; j < n; ++j)
                            C(i, j) = (M[i] - N[j]).norm();

                    const auto assign = ::hungarian<double>(C);

                    // parejas válidas con gate
                    const double GATE = 600.0; // mm
                    std::vector<std::pair<int,int>> pairs;
                    pairs.reserve(assign.size());
                    for (auto [i, j] : assign)
                        if ((int)i < m && (int)j < n && (M[i] - N[j]).norm() < GATE)
                            pairs.emplace_back((int)i, (int)j);

                    if ((int)pairs.size() >= 3)
                    {
                        // 21.d) W y b
                        Eigen::MatrixXd W(pairs.size() * 2, 3);
                        Eigen::VectorXd b(pairs.size() * 2);
                        for (size_t k = 0; k < pairs.size(); ++k)
                        {
                            const auto &pm = M[pairs[k].first];
                            const auto &pn = N[pairs[k].second];

                            b(2*k)     = pn.x() - pm.x();
                            b(2*k + 1) = pn.y() - pm.y();

                            W.block<1,3>(2*k, 0)     << 1.0, 0.0, -pm.y();
                            W.block<1,3>(2*k + 1, 0) << 0.0, 1.0,  pm.x();
                        }

                        // 21.e–f) resolver incremento y actualizar pose con ganancia y clamp
                        Eigen::Vector3d r = (W.transpose() * W).ldlt().solve(W.transpose() * b);
                        if (r.array().allFinite())
                        {
                            const double alpha_t = 0.3, alpha_r = 0.3;
                            r(0) = std::clamp(r(0), -200.0, 200.0);
                            r(1) = std::clamp(r(1), -200.0, 200.0);
                            r(2) = std::clamp(r(2), -0.30,  0.30);

                            robot_pose.pretranslate(alpha_t * Eigen::Vector2d(r(0), r(1)));
                            robot_pose.rotate(Eigen::Rotation2D<double>(alpha_r * r(2)));

                            // 21.g) actualizar dibujo izquierdo desde localización
                            if (robot_polygon)
                            {
                                robot_polygon->setPos(robot_pose.translation().x(),
                                                      robot_pose.translation().y());
                                const double angle = std::atan2(robot_pose.rotation()(1, 0),
                                                                robot_pose.rotation()(0, 0));
                                robot_polygon->setRotation(qRadiansToDegrees(angle));
                            }
                        }
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            qWarning() << "[localisation] exception:" << e.what();
        }
        catch (...)
        {
            qWarning() << "[localisation] unknown exception";
        }
    }

    // --- 3) NO enviar velocidades: JoystickPublish controla OmniRobot ---
    // (nada aquí)

    // --- 4) Derecha SIEMPRE por odometría ---
    update_right_from_odo();
}

/**
 * === FILTRADO DE LIDAR (mínimo por ángulo) ===
 */
std::optional<RoboCompLidar3D::TPoints>
SpecificWorker::filter_lidar(const RoboCompLidar3D::TPoints &points)
{
    if (points.empty()) return {};

    RoboCompLidar3D::TPoints filtered;
    for (auto &&[angle, pts] : iter::groupby(points, [](const auto &p)
    {
        float multiplier = std::pow(10.f, 2);   // redondeo a 0.01 rad
        return std::floor(p.phi * multiplier) / multiplier;
    }))
    {
        auto min_it = std::min_element(pts.begin(), pts.end(),
                                       [](const auto &a, const auto &b) { return a.r < b.r; });
        if (min_it != pts.end())
            filtered.emplace_back(*min_it);
    }
    return filtered;
}

/**
 * === DIBUJADO DE PUNTOS LIDAR (izquierda) ===
 */
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem *> draw_points;
    if (!scene) return;

    for (const auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("LightGreen");
    const QPen pen(color, 10);

    for (const auto &p : points)
    {
        auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp);
    }
}

/**
 * === ODOMETRÍA -> derecha SIEMPRE ===
 */
void SpecificWorker::update_right_from_odo()
{
    try
    {
        if (!robot_polygon_room) return;
        RoboCompGenericBase::TBaseState b;
        omnirobot_proxy->getBaseState(b);

        constexpr bool ODO_IN_METERS = true;      // ajusta si ya viene en mm
        const double x = ODO_IN_METERS ? b.x * 1000.0 : b.x;
        const double y = ODO_IN_METERS ? b.z * 1000.0 : b.z;

        robot_polygon_room->setPos(x, y);
        robot_polygon_room->setRotation(qRadiansToDegrees(b.alpha));
    }
    catch(const Ice::Exception &e)
    {
        qWarning() << e.what();
    }
}

/**
 * === DIBUJAR EL MAPA (4 paredes) ===
 */
void SpecificWorker::draw_room(QGraphicsScene* scene, const QRectF& dims)
{
    if (!scene) return;
    scene->setSceneRect(dims);
    scene->setBackgroundBrush(QColor(25,25,25));

    QPen wall(Qt::lightGray);
    wall.setWidth(3);
    wall.setCosmetic(true);

    scene->addLine(dims.left(),  dims.top(),    dims.right(), dims.top(),    wall); // arriba
    scene->addLine(dims.right(), dims.top(),    dims.right(), dims.bottom(), wall); // derecha
    scene->addLine(dims.right(), dims.bottom(), dims.left(),  dims.bottom(), wall); // abajo
    scene->addLine(dims.left(),  dims.bottom(), dims.left(),  dims.top(),    wall); // izquierda
}

/**
 * === EMERGENCIA Y RESTAURACIÓN ===
 */
void SpecificWorker::emergency() { std::cout << "Emergency worker" << std::endl; }
void SpecificWorker::restore()   { std::cout << "Restore worker"   << std::endl; }

/**
 * === STARTUP CHECK ===
 */
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}

void SpecificWorker::new_target_slot(QPointF p)
{
    Q_UNUSED(p);
}