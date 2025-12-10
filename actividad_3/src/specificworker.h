#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"

#include <expected>
#include <random>
#include <tuple>
#include <utility>
#include <vector>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <QGraphicsPolygonItem>
#include <QRectF>
#include <QGraphicsRectItem>

#include <doublebuffer/DoubleBuffer.h>
#include "time_series_plotter.h"

#ifdef emit
#undef emit
#endif
#include <execution>

#include "common_types.h"     // Corners, Match, Lines, etc.
#include "room_detector.h"    // rc::Room_Detector
#include "hungarian.h"        // rc::Hungarian
#include "nominal_room.h"     // NominalRoom
#include "door_detector.h"
#include "image_processor.h"

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker final : public GenericWorker
{
    Q_OBJECT

public:
    SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
    ~SpecificWorker();


public slots:
    void initialize() override;
    void compute() override;
    void emergency() override;
    void restore() override;
    int  startup_check();

private:
    bool startup_check_flag;

    // ============================
    // PARÁMETROS DEL ROBOT / MUNDO
    // ============================
    struct Params
    {
        float ROBOT_WIDTH  = 460.f;   // mm
        float ROBOT_LENGTH = 480.f;   // mm

        float MAX_ADV_SPEED  = 1000.f; // mm/s
        float MAX_ROT_SPEED  = 1.f;    // rad/s
        float MAX_SIDE_SPEED = 50.f;   // mm/s

        float MAX_TRANSLATION = 500.f; // mm/s
        float MAX_ROTATION    = 0.2f;  // rad/s

        float STOP_THRESHOLD    = 700.f;           // mm
        float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm

        // LIDAR secciones
        float LIDAR_FRONT_SECTION      = 0.2f;      // rad, ~12 deg
        float LIDAR_RIGHT_SIDE_SECTION =  M_PI/3.f; // rad
        float LIDAR_LEFT_SIDE_SECTION  = -M_PI/3.f; // rad

        float WALL_MIN_DISTANCE = ROBOT_WIDTH * 1.2f;

        // match error correction
        float MATCH_ERROR_SIGMA         = 150.f;  // mm
        float DOOR_REACHED_DIST         = 300.f;  // mm
        std::string LIDAR_NAME_LOW      = "bpearl";
        std::string LIDAR_NAME_HIGH     = "helios";
        QRectF GRID_MAX_DIM             = QRectF{-5000, 2500, 10000, -5000};

        // relocalization
        float RELOCAL_CENTER_EPS           = 300.f;     // radio para considerar "en el centro"
        float RELOCAL_KP                   = 0.002f;    // ganancia para control de avance (si se usa)
        float RELOCAL_MAX_ADV              = 300.f;     // mm/s
        float RELOCAL_MAX_SIDE             = 300.f;
        float RELOCAL_ROT_SPEED            = 0.6f;      // rad/s
        float RELOCAL_DELTA                = 5.0f * M_PI/180.f;
        float RELOCAL_MATCH_MAX_DIST       = 2000.f;
        float RELOCAL_DONE_COST            = 500.f;
        float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;
        float CROSS_DOOR_SPEED    = 400.f;   // mm/s
        float CROSS_DOOR_DURATION = 6.f;     // segundos
    };
    Params params;

    // ============
    // VIEWERS / UI
    // ============
    AbstractGraphicViewer *viewer      = nullptr; // izquierda
    AbstractGraphicViewer *viewer_room = nullptr; // derecha

    QGraphicsPolygonItem *robot_draw      = nullptr; // robot en viewer izquierdo
    QGraphicsPolygonItem *robot_room_draw = nullptr; // robot en viewer_room

    // =============
    // ROBOT POSE
    // =============
    Eigen::Affine2d robot_pose = Eigen::Affine2d::Identity();

    // =============
    // ROOMS & MATCH
    // =============
    std::vector<NominalRoom> nominal_rooms{
        NominalRoom{8000.f, 4000.f},
        NominalRoom{5500.f, 4000.f}
    };

    rc::Room_Detector room_detector;
    rc::Hungarian     hungarian;

    // =============
    // STATE MACHINE
    // =============
    enum class STATE
    {
        GOTO_DOOR,
        ORIENT_TO_DOOR,
        LOCALISE,
        GOTO_ROOM_CENTER,
        TURN,
        IDLE,
        CROSS_DOOR,
        UPDATE_POSE
    };

    inline const char* to_string(STATE s) const
    {
        switch (s)
        {
            case STATE::IDLE:             return "IDLE";
            case STATE::LOCALISE:         return "LOCALISE";
            case STATE::GOTO_DOOR:        return "GOTO_DOOR";
            case STATE::TURN:             return "TURN";
            case STATE::ORIENT_TO_DOOR:   return "ORIENT_TO_DOOR";
            case STATE::GOTO_ROOM_CENTER: return "GOTO_ROOM_CENTER";
            case STATE::CROSS_DOOR:       return "CROSS_DOOR";
            case STATE::UPDATE_POSE:      return "UPDATE_POSE";
            default:                      return "UNKNOWN";
        }
    }

    STATE state = STATE::LOCALISE;
    using RetVal = std::tuple<STATE, float, float>;

    RetVal goto_door(const RoboCompLidar3D::TPoints &points);
    RetVal orient_to_door(const RoboCompLidar3D::TPoints &points);
    RetVal cross_door(const RoboCompLidar3D::TPoints &points);
    RetVal localise(const Match &match);

    RetVal goto_room_center(const RoboCompLidar3D::TPoints &points, const Lines &lines);

    RetVal update_pose(const Corners &corners, const Match &match);
    RetVal turn(const Corners &corners);

    RetVal process_state(const RoboCompLidar3D::TPoints &data,
                         const Corners &corners,
                         const Lines   &lines,
                         const Match   &match,
                         AbstractGraphicViewer *viewer);

    // =============
    // DRAW
    // =============
    void draw_lidar(const RoboCompLidar3D::TPoints &filtered_points,
                    std::optional<Eigen::Vector2d> center,
                    QGraphicsScene *scene);

    // =============
    // AUX
    // =============
    RoboCompLidar3D::TPoints read_data();

    std::expected<int, std::string>
    closest_lidar_index_to_given_angle(const auto &points, float angle);

    RoboCompLidar3D::TPoints filter_same_phi        (const RoboCompLidar3D::TPoints &points);
    RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points,
                                                    float d);

    void print_match(const Match &match, float error = 1.f) const;

    // RNG
    std::random_device rd;

    // DoubleBuffer para comandos de velocidad
    DoubleBuffer<std::tuple<float, float, float, long>,
                 std::tuple<float, float, float, long>> commands_buffer;

    std::tuple<float, float, float, long> last_velocities{0.f, 0.f, 0.f, 0.f};

    // Plotter
    std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
    int match_error_graph = -1; // índice del gráfico en el plotter

    // Doors
    DoorDetector door_detector;

    // Image processor (para la parte visual, parche rojo, etc.)
    rc::ImageProcessor image_processor;

    // Timing
    std::chrono::time_point<std::chrono::high_resolution_clock>
        last_time = std::chrono::high_resolution_clock::now();

    // Relocalization flags
    bool relocal_centered   = false;
    bool localised          = false;
    bool red_patch_detected = false;
    bool crossing_door      = false;
    bool crossed_door       = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> cross_door_start;

    float door_travel_target_mm = 0.f;   // distancia total a recorrer al cruzar

    // Pose update & control
    bool update_robot_pose(const Corners &corners, const Match &match);
    void move_robot      (float adv, float rot, float max_match_error);
    Eigen::Vector3d solve_pose      (const Corners &corners, const Match &match);
    void           predict_robot_pose();
    std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);

signals:
    // void customSignal();
};

#endif // SPECIFICWORKER_H
