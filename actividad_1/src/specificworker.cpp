/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#include <iostream>
#include <qcolor.h>
#include <QRect>
#include <cppitertools/groupby.hpp>



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
    std::cout << "initialize worker" << std::endl;



	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	//this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

	//initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name")

}

/**
* TU LIDAR ESTA SIMULADO DENTRO DE WEBOTS Y CONECTADO AL SISTEMA ROBOCOMP. EL COMPONENTE LIDAR3D RECIBE LOS DATOS DEL SENSOR (EN WEBOTS).
* TU COMPONENTE CHOCACHOCA LOS PIDE POR RED (USANDO LIDAR3D_PROXY). LO QUE RECIBE ES UNA LISTA DE PUNTOS (X, Y, Z, PHI, DISTANCIA...).
*/

/*
 * Este metodo se ejecuta periodicamente con el periodo indicado en el fichero de configuración(cada "tick" del componente).
 * De esta forma decides que hace el robot en cada momento.
 */
void SpecificWorker::compute()
{
    std::vector<RoboCompLidar3D::TPoint> points;

    try {
        // Obtener los datos del LIDAR 3D
        const auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 5000, 1);
        const auto filtered = filter_lidar(data.points);
        if (filtered.has_value()) {
            draw_lidar(filtered.value(), &viewer->scene);
            points = filtered.value();
        }
    }
    catch (const Ice::Exception &e) {
        std::cout << e.what() << std::endl;
    }

    if (points.empty())
        return;

    // === CÁLCULO DE DISTANCIAS ===
    int size = points.size();
    int mid_index = size / 2;
    int left_index = size * 0.30;
    int right_index = size * 0.70;

    float frontal_dist = points[mid_index].distance2d;
    float left_dist = points[left_index].distance2d;
    float right_dist = points[right_index].distance2d;

    qInfo() << "Frontal: " << frontal_dist << " | Left: " << left_dist << " | Right: " << right_dist;

    // === VARIABLES DE MOVIMIENTO ===
    float side = 0.f;
    float adv = 0.f;
    float rot = 0.f;


    static bool is_rotating = false;
    static float rotation_direction = 1.f;
    static std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<float, std::ratio<1, 1000000000>>> rotation_end_time;

    // Umbrales
    const float OBSTACLE_THRESHOLD = 500.f;
    const float CLEAR_THRESHOLD = 800.f;

    auto now = std::chrono::steady_clock::now();

    // === LÓGICA DE DECISIÓN ACTUALIZADA ===
    bool obstacle_frontal = frontal_dist < OBSTACLE_THRESHOLD;
    bool obstacle_left = left_dist < OBSTACLE_THRESHOLD;
    bool obstacle_right = right_dist < OBSTACLE_THRESHOLD;

	if ((obstacle_frontal || obstacle_left || obstacle_right) && !is_rotating)
	{
		is_rotating = true;

		// Dirección de giro aleatoria por defecto [-1.5, 1.5]
		rotation_direction = ((std::rand() % 3000) / 1000.0f) - 1.5f;

		float rotation_duration = 0.0f;

		// Caso: SOLO obstáculo a la izquierda → gira a la derecha durante 1 segundo
		if (obstacle_left && !obstacle_frontal && !obstacle_right)
		{
			rotation_direction = ((std::rand() % 1000) / 1000.0f) * 1.5f;  // [0.0, 1.5]
			rotation_duration = 0.9f + static_cast<float>(std::rand() % 301) / 1000.f;  // [0.9, 1.2]

			//rotation_duration = 1.2f;
		}
		// Caso: SOLO obstáculo a la derecha → gira a la izquierda durante 1 segundo
		else if (obstacle_right && !obstacle_frontal && !obstacle_left)
		{
			rotation_direction = -((std::rand() % 1000) / 1000.0f) * 1.5f; // [-1.5, 0.0]
			rotation_duration = 0.9f + static_cast<float>(std::rand() % 301) / 1000.f;  // [0.9, 1.2]

			//rotation_duration = 1.2f;
		}
		// Caso: obstáculo FRONTAL (con o sin laterales) → duración aleatoria (1.0 a 2.0 s)
		else
		{
			rotation_duration = 0.9f + static_cast<float>(std::rand() % 301) / 1000.f;  // [0.9, 1.2]

			//rotation_duration = 1.f + static_cast<float>(std::rand() % 1200) / 1200.f;  // [1.5, 3.0]
		}

		rotation_end_time = now + std::chrono::duration<float>(rotation_duration);

		std::cout << "Iniciando giro por obstáculo. Dirección: " << rotation_direction
				  << " | Duración: " << rotation_duration << " segundos" << std::endl;
	}


    // Si está girando (por obstáculo frontal o lateral)
    if (is_rotating)
    {
        if (now < rotation_end_time)
        {
            adv = 0.f;
            rot = 1.2f;
        }
        else
        {
            is_rotating = false;
        }
    }
    // Camino despejado → avanza
    else if (frontal_dist > CLEAR_THRESHOLD)
    {
        adv = 1000.f;

        // Pequeña rotación aleatoria para evitar trayectoria recta
        //float small_rotation = ((std::rand() % 100) / 100.0f - 0.5f) * 0.2f;
        rot = 0.f;
    }
    // Zona intermedia: avanzar lento
    else
    {
        adv = 1000.f;
        rot = 0.f;
    }

    // === ENVIAR VELOCIDADES AL ROBOT ===
    try
    {
        omnirobot_proxy->setSpeedBase(side, adv, rot);
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e.what() << std::endl;
    }
}




//////////////////////////////////////////////////////////////////
// RoboCompLidar3D: nombre de la interfaz dentro del sistema RoboComp que gestiona los sensores LIDAR 3D.
/*
* Le pasamos por parametro la lista de puntos obtenida a traves del proxy del lidar3d. Cada punto representa una lectura del sensor LIDAR
* Cada punto tiene los siguientes campos: x, y, z, phi, theta, r
*
* Como con lidar3d tenemos mucha informacion y ruido debido a que hay demasiados datos, puntos repetidos etc el objetivo de este metodo
* es quedarse solo con los puntos mas relevantes. En este caso nos quedamos con el punto mas cercano (minimo r) de cada angulo phi.
*/
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_lidar(const RoboCompLidar3D::TPoints &points)
{
	if (points.empty()) return {};

	//Creamos un nuevo vector donde iremos aniadiendo los puntos que nos interesan
	RoboCompLidar3D::TPoints filtered;
	// phi representa el angulo redondeado a 2 decimales, por ejemplo 0.52
	// pts tendra todos los puntos que tienen ese mismo angulo phi: 0.520, 0.524, 0.529... y los vamos agrupando con groupby
	for (auto &&[angle, pts] : iter::groupby(points, [](const auto &p)
		{
			//redondeamos a dos decimales
			float multiplier = std::pow(10.f, 2);
			return std::floor(p.phi*multiplier)/multiplier;
		}))
	{		// esto a calcular el punto con menor r (distancia), devolviendo un iterador que APUNTA al punto con menor r,
			// por tanto luego hacemos un 'emplace_back' para aniadir ese punto al vector final, que tendra el valor minimo de cada grupo.
			auto min_it = std::min_element(pts.begin(), pts.end(), [](const auto &a, const auto &b)
					{return a.r < b.r;} );
			filtered.emplace_back(*min_it);
	}
	return filtered;
}
/*
* La funcion draw_lidar dibuja los puntos del lidar en la escena del viewer.
* Le pasamos como parametro la lista de puntos que queremos dibujar y la escena donde se dibuja.
*/
void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
	// static para que mantenga el valor entre llamadas ya que  este metodo se ejecuta muchas veces por segundo, en cada tick del metodo compute
	/*
	* Cada vez:
	* - Recibe nuevos puntos del LiDAR
	* - Dibuja los puntos por pantalla
	* - Borra los puntos dibujados la vez anterior(los del tick anterior)
	*/
	static std::vector<QGraphicsItem*> draw_points;
	// recorre todos los puntos, los elimina de la escena y los borra de memoria
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();
	// color de los puntos
	const QColor color("LightGreen");
	// creamos un lapiz de color y grosor 10
	const QPen pen(color, 10);
	// vamos a dibujar el cuadrado
	for (const auto &p : points)
	{
		// crea un cuadrado verde de 50x50 centrado en (0,0). Lo de centrado lo conseguimos poniendo -25,-25 (la esquina superior izquierda)
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		// mueve ese cuadrado a la posicion real del punto detectado
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
	}
}

// Este metodo actualiza la posicion del robot en el viewer
void SpecificWorker::update_robot_position() {
	try {
		// Crea una 'foto' del estado actual del robot
		// bState tiene los campos x, z, alpha (posicion y orientacion
		RoboCompGenericBase::TBaseState bState;
		// 'omnirobot_proxy' es el proxy que nos permite comunicarnos con el componente RoboCompOmniRobot( el que controla las ruedas del robot)
		omnirobot_proxy->getBaseState(bState);
		// la rotacion la hacemos en radianes por eso la cuenta esa que hay entre parentesis
		robot_polygon->setRotation(bState.alpha*180/M_1_PI);
		// mueve el robot a la posicion x,z (ojo que en el viewer z es y, ya que es 2D)
		robot_polygon->setPos(bState.x,bState.z);
		std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
	}
	catch (const Ice::Exception &e)  {std::cout << e.what() << std::endl;
	}
}

///
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}


void SpecificWorker::new_target_slot(QPointF p){

}

/**
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppintertools(const RoboCompLidar3D::TPoints& points)
{
	if (points.empty())
		return {};

	RoboCompLidar3D::TPoints result; result.reserve(points.size());

	//Loop over the groups produced by iter::gorupby
	for (auto && [angle, group] : iter::groupby(points, [](const auto &p) {
		float multiplier = std::pow(10.0f, 2); return std::floor(p.phi * multiplier) /
	}))

}
*/

/**************************************/
// From the RoboCompLidar3D you can call this methods:
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