// specificworker.h
//
// Cabecera del componente (SpecificWorker).
// Aquí se declaran:
//
//  - Parámetros del robot/mundo (struct Params)
//  - Viewers (UI): izquierda (sensor/robot) y derecha (mapa/sala nominal)
//  - Estado interno: pose, salas nominales, flags, buffers
//  - Máquina de estados (enum STATE) + métodos de cada estado
//  - Utilidades: lectura LiDAR, filtros, dibujo, control, actualización de pose
//
// La idea general del componente es:
//  - Leer LiDAR -> extraer geometría (esquinas/lineas)
//  - Matching contra una sala nominal (Hungarian)
//  - Usar una máquina de estados para ir al centro, buscar puerta, orientar y cruzar
//  - Dibujar debug en dos frames de Qt.

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// Si quieres reducir el periodo automáticamente por falta de uso, descomenta esto.
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"

#include <expected>   // std::expected (C++23/libstdc++ reciente)
#include <random>
#include <tuple>
#include <utility>
#include <vector>
#include <chrono>

#include <QLCDNumber>
#include <QLabel>
#include <QtMath>


#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <QGraphicsPolygonItem>
#include <QRectF>

#include <QGraphicsRectItem>
#include <limits>

// DoubleBuffer: normalmente se usa para pasar datos entre hilos o “logging”
// sin bloquear (producer/consumer). Aquí se usa para guardar comandos de velocidad.
#include <doublebuffer/DoubleBuffer.h>

// Plotter de series temporales (para depurar error en el tiempo).
#include "time_series_plotter.h"

// En algunos entornos `emit` puede colisionar con macros (por Qt),
// por eso lo undefinean antes de incluir ciertos headers.
#ifdef emit
#undef emit
#endif
#include <execution>  // std::execution (para paralelismo; aquí solo se declara)

#include "common_types.h"     // Tipos comunes del proyecto: Corners, Match, Lines, etc.
#include "room_detector.h"    // rc::Room_Detector (detección de esquinas/lineas)
#include "hungarian.h"        // rc::Hungarian (matching óptimo)
#include "nominal_room.h"     // NominalRoom (sala ideal/nominal)
#include "door_detector.h"    // DoorDetector + Door(s)
#include "image_processor.h"  // rc::ImageProcessor (parte visual; aquí casi no se usa)

/**
 * \brief SpecificWorker: lógica principal del componente RoboComp.
 *
 * Hereda de GenericWorker, que te da:
 *  - proxies (lidar3d_proxy, omnirobot_proxy, etc.)
 *  - slots Qt: initialize(), compute(), emergency(), restore()
 *  - acceso a los frames de la UI (frame, frame_room, frame_plot_error, ...)
 *
 * En esta clase es donde implementas:
 *  - el pipeline del LiDAR + detección
 *  - la máquina de estados de navegación
 *  - la parte de dibujo/debug en Qt
 */
class SpecificWorker final : public GenericWorker
{
    Q_OBJECT

public:
    // Constructor:
    //  - configLoader: carga del fichero etc/config
    //  - tprx: tupla de proxies (Ice)
    //  - startup_check: si está a true, el componente no arranca el bucle normal
    SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
    ~SpecificWorker();

public slots:
    // initialize(): se llama 1 vez al arrancar (UI, pose inicial, plotter, stop robot)
    void initialize() override;

    // compute(): bucle principal periódico (leer LiDAR, procesar, decidir control, enviar velocidades)
    void compute() override;

    // emergency()/restore(): hooks estándar RoboComp
    void emergency() override;
    void restore() override;

    // startup_check(): modo de verificación (suele terminar el proceso a los pocos ms)
    int startup_check();

private:
    // Flag interno para saber si se arrancó en modo startup_check
    bool startup_check_flag;

	bool mnist_checked_in_this_room = false;

    std::chrono::high_resolution_clock::time_point turn_start_time;
    std::chrono::high_resolution_clock::time_point last_mnist_check;

	std::chrono::high_resolution_clock::time_point no_center_start_time;
	bool rotating_to_find_center = false;



    // ============================
    // PARÁMETROS DEL ROBOT / MUNDO
    // ============================
    // Contiene constantes/umbrales de control y de geometría.
    // Se usa desde muchos métodos: compute(), goto_room_center(), goto_door(), etc.
    struct Params
    {
        // Dimensiones robot (mm)
        float ROBOT_WIDTH  = 460.f;
        float ROBOT_LENGTH = 480.f;

        // Límites generales de velocidad (mm/s, rad/s)
        float MAX_ADV_SPEED  = 1000.f;
        float MAX_ROT_SPEED  = 1.f;
        float MAX_SIDE_SPEED = 50.f;

        // Límites usados en otras partes (ojo: hay “duplicados” conceptuales)
        float MAX_TRANSLATION = 500.f;
        float MAX_ROTATION    = 0.2f;

        // Umbrales “reactivos”
        float STOP_THRESHOLD    = 700.f;            // mm, para parar si algo está cerca
        float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3;  // mm, margen para “avanzar”

        // Secciones del LiDAR en radianes
        // (útil si haces lógica por sectores frontal/lados)
        float LIDAR_FRONT_SECTION      = 0.2f;       // ~12°
        float LIDAR_RIGHT_SIDE_SECTION =  M_PI/3.f;
        float LIDAR_LEFT_SIDE_SECTION  = -M_PI/3.f;

        // Distancia mínima para considerar pared “válida”
        float WALL_MIN_DISTANCE = ROBOT_WIDTH * 1.2f;

        // Parámetros de matching / localización
        float MATCH_ERROR_SIGMA         = 150.f;   // mm (para ponderar/corregir)
        float DOOR_REACHED_DIST         = 300.f;   // mm: cerca del “pre-door target”
        std::string LIDAR_NAME_LOW      = "bpearl";
        std::string LIDAR_NAME_HIGH     = "helios";

        // Dimensiones del “mundo” que pinta el viewer (rect del scene)
        QRectF GRID_MAX_DIM             = QRectF{-5000, 2500, 10000, -5000};

        // ===== Relocalization =====
        // eps: “radio” para decir que estás centrado
        float RELOCAL_CENTER_EPS           = 300.f;

        // Ganancia “kp” (si se usara control proporcional por distancia)
        float RELOCAL_KP                   = 0.002f;

        // Velocidades de relocalización (mm/s, rad/s)
        float RELOCAL_MAX_ADV              = 300.f;
        float RELOCAL_MAX_SIDE             = 300.f;
        float RELOCAL_ROT_SPEED            = 0.6f;

        // Delta angular usado en algunos algoritmos (p. ej. búsqueda)
        float RELOCAL_DELTA                = 5.0f * M_PI/180.f;

        // Umbrales para matching / done
        float RELOCAL_MATCH_MAX_DIST       = 2000.f;
        float RELOCAL_DONE_COST            = 500.f;
        float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;

        // ===== Cruce de puertas =====
        // En la versión “por tiempo”, cruzas recto durante CROSS_DOOR_DURATION
        float CROSS_DOOR_SPEED    = 400.f;  // mm/s
        float CROSS_DOOR_DURATION = 3.f;    // s
    };
    Params params;

    // ============
    // VIEWERS / UI
    // ============
    // `viewer` (izquierda) suele ser “sensor view”: LiDAR + robot en frame local.
    // `viewer_room` (derecha) suele ser “map view”: sala nominal + robot con pose estimada.
    AbstractGraphicViewer* viewer      = nullptr; // izquierda
    AbstractGraphicViewer* viewer_room = nullptr; // derecha

    // Representación del robot en cada viewer (un polígono en escena Qt)
    QGraphicsPolygonItem* robot_draw      = nullptr; // robot en viewer
    QGraphicsPolygonItem* robot_room_draw = nullptr; // robot en viewer_room


    // =============
    // ROOM VIEW (viewer derecho)
    // =============
    int current_room_idx = 0;
    QGraphicsRectItem *room_rect_draw = nullptr;

    //Detección de números
    int last_detected_number = -1;
    float last_detected_confidence = 0.f;

    // ===== Detección de números MNIST =====
    bool number_read_in_this_wall = false;

    // Helpers para actualizar el mapa derecho
    void update_room_rect();

    void update_debug_panel(float adv, float rot);

    // Selección inicial de sala
    struct RoomMatchResult
    {
        int   room_idx   = -1;
        Match match;
        float max_error  = std::numeric_limits<float>::infinity();
    };

    RoomMatchResult find_best_room(const Corners &corners,
                                   const Eigen::Affine2d &robot_pose);


    // =============
    // ROBOT POSE
    // =============
    // Transformación 2D (rotación + traslación).
    // Se usa para dibujar el robot en el mapa y para transformar esquinas nominales.
    Eigen::Affine2d robot_pose = Eigen::Affine2d::Identity();

    // =============
    // ROOMS & MATCH
    // =============
    // Vector de salas nominales: “plantillas” de habitaciones.
    // En este ejemplo hay 2 salas:
    //  - sala 0: 5500 x 4000
    //  - sala 1: 8000 x 4000
    // (Luego tu lógica decide cuál es la actual).
    std::vector<NominalRoom> nominal_rooms{
        NominalRoom{5500.f, 4000.f},
        NominalRoom{8000.f, 4000.f}
    };

    // Detecta esquinas y paredes a partir del LiDAR
    rc::Room_Detector room_detector;

    // Hungarian algorithm: matching óptimo entre esquinas detectadas y nominales
    rc::Hungarian hungarian;

    // =============
    // STATE MACHINE
    // =============
    // Enumeración de estados de navegación.
    // El flujo típico aquí:
    //  LOCALISE -> GOTO_ROOM_CENTER -> GOTO_DOOR -> ORIENT_TO_DOOR -> CROSS_DOOR -> GOTO_ROOM_CENTER -> ...
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

    // Helper para logs/debug: estado -> string
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

    // Estado inicial: por defecto empieza localizando
    STATE state = STATE::LOCALISE;

    // Tipo de retorno de cada estado:
    //  - next_state
    //  - adv (avance, mm/s)
    //  - rot (rotación, rad/s)
    using RetVal = std::tuple<STATE, float, float>;

    // ---------- Métodos “por estado” ----------
    // Cada uno implementa la acción de un estado concreto.
    RetVal goto_door(const RoboCompLidar3D::TPoints& points);
    RetVal orient_to_door(const RoboCompLidar3D::TPoints& points);
    RetVal cross_door(const RoboCompLidar3D::TPoints& points);

    RetVal localise(const Lines& lines);
    RetVal goto_room_center(const RoboCompLidar3D::TPoints& points, const Lines& lines);
    RetVal update_pose(const Corners& corners, const Match& match);
    RetVal turn(const Corners& corners);

    // Dispatcher: elige qué método llamar en base al estado actual
    RetVal process_state(const RoboCompLidar3D::TPoints& data,
                         const Corners& corners,
                         const Lines&   lines,
                         const Match&   match,
                         AbstractGraphicViewer* viewer);

    // =============
    // DRAW
    // =============
    // Dibuja puntos LiDAR y centro estimado sobre un QGraphicsScene
    void draw_lidar(const RoboCompLidar3D::TPoints& filtered_points,
                    std::optional<Eigen::Vector2d> center,
                    QGraphicsScene* scene);

    // =============
    // AUX (sensado / filtrado / debug)
    // =============
    // Lee datos LiDAR (incluye filtros).
    RoboCompLidar3D::TPoints read_data();

    // Devuelve el índice del punto LiDAR con phi más cercano al ángulo dado.
    // Se usa para seleccionar medidas “en una dirección”.
    std::expected<int, std::string>
    closest_lidar_index_to_given_angle(const auto& points, float angle);

    // Filtrado: quedarse con el punto más cercano por cada phi (evita duplicados)
    RoboCompLidar3D::TPoints filter_same_phi(const RoboCompLidar3D::TPoints& points);

    // Filtrado: eliminar puntos aislados (ahora mismo puede ser stub)
    RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints& points,
                                                    float d);

    // Debug del matching (si lo implementas, imprimes parejas y errores)
    void print_match(const Match& match, float error = 1.f) const;

    // RNG (random device): útil si necesitas elegir acciones aleatorias o muestrear.
    std::random_device rd;

    // DoubleBuffer para comandos de velocidad:
    // Guarda tu historial de comandos (timestamp incluido) sin bloquear el hilo.
    DoubleBuffer<std::tuple<float, float, float, long>,
                 std::tuple<float, float, float, long>> commands_buffer;

    // Último comando enviado: (side, adv, rot, timestamp_ms)
    std::tuple<float, float, float, long> last_velocities{0.f, 0.f, 0.f, 0.f};

    // ===== Plotter (error de matching vs tiempo) =====
    std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
    int match_error_graph = -1;  // índice del gráfico dentro del plotter

    // ===== Detección de puertas =====
    DoorDetector door_detector;

    // Alterna el lado preferido al elegir puerta (izquierda/derecha)
    bool next_use_left = true;

    // Procesado de imagen (si tu pipeline visual lo usa; aquí está preparado)
    rc::ImageProcessor image_processor;

    // Timing general (para medir periodos o duraciones)
    std::chrono::time_point<std::chrono::high_resolution_clock>
        last_time = std::chrono::high_resolution_clock::now();

    // ===== Flags de relocalización / navegación =====
    bool relocal_centered   = false;  // “he llegado al centro”
    bool localised          = false;  // “estoy localizado” según heurística de matching
    bool red_patch_detected = false;  // placeholder/compatibilidad (no usado ahora)

    // Flags de cruce (sobran un poco con la versión por tiempo, pero no estorban)
    bool crossing_door = false;
    bool crossed_door  = false;


    bool room_view_fitted_once = false;

    // Momento de inicio del cruce (si se usa por tiempo)
    std::chrono::time_point<std::chrono::high_resolution_clock> cross_door_start;

    // Target de distancia en mm (otra variante de cruce; ahora no usado)
    float door_travel_target_mm = 0.f;

    // ==========================
    // Pose update & control
    // ==========================
    // Actualiza la pose usando corners+match (Task 2/3): normalmente calcula delta pose
    bool update_robot_pose(const Corners& corners, const Match& match);

    // Enviar velocidades al robot + guardar en buffer
    void move_robot(float adv, float rot, float max_match_error);

    // Resuelve (x,y,theta) que mejor alinea medido vs nominal (stub si no implementado)
    Eigen::Vector3d solve_pose(const Corners& corners, const Match& match);

    // Predicción (odometría) si se implementa; ahora stub
    void predict_robot_pose();

    // Controlador de navegación hacia un target (devuelve {adv, rot})
    std::tuple<float, float> robot_controller(const Eigen::Vector2f& target);

signals:
    // Señales personalizadas Qt si las necesitas.
    // void customSignal();
};

#endif // SPECIFICWORKER_H
