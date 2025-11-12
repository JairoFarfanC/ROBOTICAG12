#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <QObject>

#include <QWidget>
#include <QFrame>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

#include <tuple>
#include <optional>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "common_types.h"
#include "room_detector.h"
#include "hungarian.h"

// --- Sala nominal (paso 18) ---
struct NominalRoom
{
    float width;    // mm
    float length;   // mm
    Corners corners;

    explicit NominalRoom(float width_ = 10000.f, float length_ = 5000.f, Corners corners_ = {})
        : width(width_), length(length_), corners(std::move(corners_)) {}

    // Transforma las esquinas con una SE(2) (para room->robot: pasa robot_pose.inverse())
    Corners transform_corners_to(const Eigen::Affine2d &T) const
    {
        Corners out;
        out.reserve(corners.size());
        for (const auto &[p, a, q] : corners)
        {
            Eigen::Vector2d ep{p.x(), p.y()};
            Eigen::Vector2d tp = T * ep;  // Affine2d aplica rot+tras
            out.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, a, q);
        }
        return out;
    }
};

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
    Q_OBJECT
public:
    explicit SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx, bool startup_check);
    ~SpecificWorker();

public slots:
    void initialize();
    void compute();
    void emergency();
    void restore();
    int startup_check();
    void new_target_slot(QPointF);

private:
    // === Herramientas gráficas y variables internas ===
    bool startup_check_flag = false;
    QRectF dimensions;

    AbstractGraphicViewer *viewer = nullptr;
    AbstractGraphicViewer *viewer_room = nullptr;
    QWidget *container = nullptr;

    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon = nullptr;
    QGraphicsPolygonItem *robot_polygon_room = nullptr;

    // Localización en la sala
    Eigen::Affine2d robot_pose = Eigen::Affine2d::Identity();
    rc::Room_Detector room_detector;

    // Para dibujar
    QGraphicsPolygonItem *robot_room_draw = nullptr;

    // Sala nominal (10x5 m centrada en (0,0))
    NominalRoom room{
        10000.f, 5000.f,
        Corners{
            {QPointF{-5000.f, -2500.f}, 0.f, 0.f},
            {QPointF{5000.f, -2500.f}, 0.f, 0.f},
            {QPointF{5000.f, 2500.f}, 0.f, 0.f},
            {QPointF{-5000.f, 2500.f}, 0.f, 0.f}
        }
    };

    // === Funciones auxiliares ===
    void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
    void draw_room(QGraphicsScene *scene, const QRectF &dims);
    void update_right_from_odo();
    std::optional<RoboCompLidar3D::TPoints> filter_lidar(const RoboCompLidar3D::TPoints &points);
};

#endif