#include "specificworker.h"

#include <iostream>
#include <qcolor.h>
#include <QRect>
#include <cppitertools/groupby.hpp>
#include <QLoggingCategory>
#include <sys/socket.h>

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
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    viewer->show();
    const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
    robot_polygon = std::get<0>(rob);

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    

    current_mode = Mode::IDLE;
}

/**
 * Método compute — ejecutado periódicamente según el periodo configurado
 */
void SpecificWorker::compute()
{
    // read_data();
    // state_machine.update();
    // send_velocity_commands();
    // auto corners = extract_corners(lidar_data);


}

/**
 * === MODO IDLE ===
 */
std::tuple<SpecificWorker::Mode, float, float, float> SpecificWorker::mode_idle(float frontal, float left, float right)
{
    const float CLEAR_THRESHOLD = 800.f;

    if (frontal > CLEAR_THRESHOLD)
    {
        qInfo() << "Switching to FORWARD mode";
        return {Mode::FORWARD, 0.f, 1000.f, 0.f};
    }
    else
        return {Mode::IDLE, 0.f, 0.f, 0.f};
}

/**
 * === MODO FORWARD ===
 */
std::tuple<SpecificWorker::Mode, float, float, float>
SpecificWorker::mode_forward(float frontal, float left, float right)
{
    const float OBSTACLE_THRESHOLD = 500.f;
    const float SPIRAL_THRESHOLD = 800.f;

    bool obstacle_front = frontal < OBSTACLE_THRESHOLD;
    bool obstacle_side = (left < OBSTACLE_THRESHOLD || right < OBSTACLE_THRESHOLD);

    if (obstacle_front || obstacle_side)
    {
        qInfo() << "Obstacle detected -> Switching to TURN mode";
        return {Mode::TURN, 0.f, 0.f, 1.2f};
    }

    if (frontal > SPIRAL_THRESHOLD && left > SPIRAL_THRESHOLD && right > SPIRAL_THRESHOLD)
    {
        qInfo() << "Wide open space -> Switching to SPIRAL mode";
        return {Mode::SPIRAL, 0.f, 1000.f, 0.3f};
    }

    return {Mode::FORWARD, 0.f, 1000.f, 0.f};
}

/**
 * === MODO TURN ===
 */
std::tuple<SpecificWorker::Mode, float, float, float>
SpecificWorker::mode_turn(float frontal, float left, float right)
{
    static auto turn_start = std::chrono::steady_clock::now();
    static float rotation_duration = 1.0f;
    static float rotation_direction = 1.f;
    static bool initialized = false;

    const float OBSTACLE_THRESHOLD = 500.f;
    const float CLEAR_THRESHOLD = 800.f;

    if (!initialized)
    {
        turn_start = std::chrono::steady_clock::now();

        rotation_duration = 0.4f + static_cast<float>(std::rand() % 301) / 1000.f; // 0.4–0.7

        if (left < OBSTACLE_THRESHOLD && right > OBSTACLE_THRESHOLD)
            rotation_direction = 1.f; // Obstáculo a la izquierda → girar a la derecha
        else if (right < OBSTACLE_THRESHOLD && left > OBSTACLE_THRESHOLD)
            rotation_direction = -1.f; // Obstáculo a la derecha → girar a la izquierda
        else if (frontal < OBSTACLE_THRESHOLD)
            rotation_direction = (std::rand() % 2 == 0) ? 1.f : -1.f;

        initialized = true;
        qInfo() << "TURN mode: direction =" << rotation_direction
                << " duration =" << rotation_duration << "s";
    }

    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - turn_start).count();

    if (elapsed < rotation_duration)
        return {Mode::TURN, 0.f, 0.f, 1.2f * rotation_direction};

    if (frontal > CLEAR_THRESHOLD)
    {
        initialized = false;
        qInfo() << "Area libre detectada → pasando a FORWARD";
        return {Mode::FORWARD, 0.f, 1000.f, 0.f};
    }
    else
    {
        turn_start = now;
        rotation_duration = 0.3f + static_cast<float>(std::rand() % 301) / 1000.f; // 0.4–0.7 s
        qInfo() << "Obstáculo aún presente → continuando giro";
        return {Mode::TURN, 0.f, 0.f, 1.2f * rotation_direction};
    }
}

/**
 * === MODO SPIRAL ===
 */
std::tuple<SpecificWorker::Mode, float, float, float>
SpecificWorker::mode_spiral(float frontal, float left, float right)
{
    const float OBSTACLE_THRESHOLD = 500.f;
    const float ADV_SPEED = 1000.f;
    const float ROT_SPEED = 0.3f;

    static bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        qInfo() << "Entering SPIRAL mode";
    }

    bool obstacle_front = frontal < OBSTACLE_THRESHOLD;
    bool obstacle_side = (left < OBSTACLE_THRESHOLD || right < OBSTACLE_THRESHOLD);

    if (obstacle_front || obstacle_side)
    {
        initialized = false;
        qInfo() << "Obstacle detected -> switching to TURN";
        return {Mode::TURN, 0.f, 0.f, 1.2f};
    }

    qInfo() << "SPIRAL mode | adv:" << ADV_SPEED << " rot:" << ROT_SPEED;
    return {Mode::SPIRAL, 0.f, ADV_SPEED, ROT_SPEED};
}

/**
 * === FILTRADO DE LIDAR ===
 */
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_lidar(const RoboCompLidar3D::TPoints &points)
{
    if (points.empty()) return {};

    RoboCompLidar3D::TPoints filtered;
    for (auto &&[angle, pts] : iter::groupby(points, [](const auto &p)
    {
        float multiplier = std::pow(10.f, 2);
        return std::floor(p.phi * multiplier) / multiplier;
    }))
    {
        auto min_it = std::min_element(pts.begin(), pts.end(),
                                       [](const auto &a, const auto &b) { return a.r < b.r; });
        filtered.emplace_back(*min_it);
    }
    return filtered;
}

/**
 * === DIBUJADO DE PUNTOS LIDAR ===
 */
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem *> draw_points;

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
 * === ACTUALIZAR POSICIÓN DEL ROBOT EN EL VIEWER ===
 */
void SpecificWorker::update_robot_position()
{
    try
    {
        RoboCompGenericBase::TBaseState bState;
        omnirobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha * 180 / M_1_PI);
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e.what() << std::endl;
    }
}

/**
 * === EMERGENCIA Y RESTAURACIÓN ===
 */
void SpecificWorker::emergency() { std::cout << "Emergency worker" << std::endl; }
void SpecificWorker::restore() { std::cout << "Restore worker" << std::endl; }

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
