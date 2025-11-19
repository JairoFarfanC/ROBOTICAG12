#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <QObject>
#include <qtmetamacros.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <tuple>
#include <optional>
#include <chrono>

#include <QGraphicsPolygonItem>
#include <QPointF>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "common_types.h"     // Corners, Lines, etc.
#include "room_detector.h"    // rc::Room_Detector

// =====================================================================
// NominalRoom: sala ideal de la TASK 2 (del documento)
// =====================================================================
struct NominalRoom
{
    float   width;    // mm
    float   length;   // mm
    Corners corners;  // esquinas nominales en el frame de la sala

    explicit NominalRoom(float width_  = 10000.f,
                         float length_ = 5000.f,
                         Corners corners_ = {})
        : width(width_)
        , length(length_)
        , corners(std::move(corners_))
    {}

    /// Transforma las esquinas nominales con una transformación SE(2)
    /// Para pasar de room -> robot se usa el inverso de robot_pose.
    [[nodiscard]]
    Corners transform_corners_to(const Eigen::Isometry2d &transform) const
    {
        Corners transformed_corners;
        transformed_corners.reserve(corners.size());

        for (const auto &[p, _, __] : corners)
        {
            Eigen::Vector2d ep{p.x(), p.y()};
            Eigen::Vector2d tp = transform * ep;   // aplica traslación + rotación

            transformed_corners.emplace_back(
                QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())},
                0.f,
                0.f
            );
        }
        return transformed_corners;
    }
};

// Sala nominal de 10x5 m centrada en el origen (TASK 2)
static const NominalRoom DEFAULT_ROOM{
    10000.f, 5000.f,
    {
        { QPointF{-5000.f, -2500.f}, 0.f, 0.f },
        { QPointF{ 5000.f, -2500.f}, 0.f, 0.f },
        { QPointF{ 5000.f,  2500.f}, 0.f, 0.f },
        { QPointF{-5000.f,  2500.f}, 0.f, 0.f }
    }
};

// =====================================================================
// SpecificWorker
// =====================================================================

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
public:
    explicit SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
    ~SpecificWorker();

public slots:
    void initialize() override;
    void compute() override;
    void emergency() override;
    void restore() override;
    int  startup_check();
    void new_target_slot(QPointF);
    void update_robot_position();   // de momento la puedes dejar vacía

private:

    // === ENUM de modos del robot (para TASK 3) ===
    enum class Mode { IDLE, FORWARD, TURN, SPIRAL };
    Mode current_mode = Mode::IDLE;

    // === Métodos de comportamiento (se rellenarán en TASK 3) ===
    std::tuple<Mode, float, float, float> mode_idle   (float frontal, float left, float right);
    std::tuple<Mode, float, float, float> mode_forward(float frontal, float left, float right);
    std::tuple<Mode, float, float, float> mode_turn   (float frontal, float left, float right);
    std::tuple<Mode, float, float, float> mode_spiral (float frontal, float left, float right);

    // === Herramientas gráficas y variables internas ===
    bool   startup_check_flag = false;
    QRectF dimensions;                         // rectángulo de dibujo (mm)

    // visor izquierda (LiDAR + localización)
    AbstractGraphicViewer *viewer = nullptr;
    QGraphicsPolygonItem  *robot_polygon = nullptr;

    // visor derecha (mapa fijo de la sala)
    AbstractGraphicViewer *viewer_room = nullptr;
    QGraphicsPolygonItem  *robot_polygon_room = nullptr;

    const int ROBOT_LENGTH = 400;              // lado del cuadrado del robot en mm

    // === Localización (pose estimada del robot en el mundo) ===
    Eigen::Isometry2d robot_pose;              // se pone a Identity en initialize()

    // === Objetos de la TASK 2 ===
    rc::Room_Detector room_detector;           // detector de esquinas (namespace rc)
    NominalRoom       room = DEFAULT_ROOM;     // sala nominal

    // === Funciones auxiliares ===
    void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
    std::optional<RoboCompLidar3D::TPoints> filter_lidar(const RoboCompLidar3D::TPoints &points);

    void update_right_from_odo();                                // usada en compute()
    void draw_room(QGraphicsScene *scene, const QRectF &dims);   // usada en initialize()


    // === Parámetros de control (mm, rad/s) ===
    const float DTH    = 500.f;   // obstáculo
    const float DCLEAR = 800.f;   // espacio libre
    const float DSP    = 800.f;   // entrar en SPIRAL
    const float VMAX   = 1000.f;  // mm/s
    const float W_TURN = 1.2f;    // rad/s
    const float W_SP   = 0.3f;    // rad/s

    // ángulos de sectores del LiDAR
    const float FRONT_HALF_ANGLE = 0.35f;        // ~±20°
    const float SIDE_HALF_ANGLE  = 0.35f;        // ventana lateral

    // devuelve (frontal, left, right) en mm
    std::tuple<float,float,float>
    lidar_distances(const RoboCompLidar3D::TPoints &points);



};

#endif