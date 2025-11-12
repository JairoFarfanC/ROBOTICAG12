
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

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
	Q_OBJECT
public:
	explicit SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
	~SpecificWorker();

public slots:
	//void connect(AbstractGraphicViewer * viewer, void(AbstractGraphicViewer::* new_mouse_coordinates)(QPointF), SpecificWorker * specific_worker, void(SpecificWorker::* new_target_slot)(QPointF)){};

	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();
	void new_target_slot(QPointF);
	void update_robot_position();

private:

	// === ENUM de modos del robot ===
	enum class Mode { IDLE, FORWARD, TURN, SPIRAL };
	Mode current_mode = Mode::IDLE;

	// === Métodos de comportamiento ===
	std::tuple<Mode, float, float, float> mode_idle(float frontal, float left, float right);
	std::tuple<Mode, float, float, float> mode_forward(float frontal, float left, float right);
	std::tuple<Mode, float, float, float> mode_turn(float frontal, float left, float right);
	std::tuple<Mode, float, float, float> mode_spiral(float frontal, float left, float right);


	// === Herramientas gráficas y variables internas ===
	bool startup_check_flag = false;
	QRectF dimensions;
	AbstractGraphicViewer *viewer = nullptr;
	const int ROBOT_LENGTH = 400;
	QGraphicsPolygonItem *robot_polygon = nullptr;

	// === Funciones auxiliares ===
	void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
	std::optional<RoboCompLidar3D::TPoints> filter_lidar(const RoboCompLidar3D::TPoints &points);
};

#endif
