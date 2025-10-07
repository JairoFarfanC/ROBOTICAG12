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
#include <stdexcept>



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

    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name")

}

/*
 *
 */
void SpecificWorker::compute()
{
    std::cout << "Compute worker" << std::endl;

    // Bucle infinito de control
    while (true)
    {
        try
        {
            // Obtener los datos del LIDAR 3D
            auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 10000, 1);
            qInfo() << "LIDAR Data Points: " << data.points.size();

            // Filtrar puntos quedándose con el mínimo de las theta iguales
            std::vector<RoboCompLidar3D::TPoint> filteredPoints;
            float minDistance = std::numeric_limits<float>::infinity();

            RoboCompLidar3D::TPoint closestPoint;

            // Filtrar puntos y encontrar el más cercano
            for (const auto& point : data.points)
            {
                bool alreadyExists = false;
                for (const auto& filteredPoint : filteredPoints)
                {
                    if (fabs(filteredPoint.theta - point.theta) < 0.01)  // Umbral para decidir que son "similares"
                    {
                        alreadyExists = true;
                        break;
                    }
                }

                if (!alreadyExists)
                {
                    filteredPoints.push_back(point);

                    // Si estamos usando coordenadas cartesianas (x, y, z):
                    float distance = sqrt(point.x * point.x + point.y * point.y);  // Distancia al origen

                    // Encontrar el punto más cercano
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        closestPoint = point;
                    }
                }
            }

        	qInfo() << minDistance;


            // Verificar si la distancia mínima está dentro del umbral seguro
            if (minDistance < 350)  // Asumimos que 100 es un umbral seguro para la distancia
            {

                // Si la distancia es muy corta, detenerse y girar
                qInfo() << "Close to obstacle, stopping and rotating...";
                omnirobot_proxy->setSpeedBase(0, 0, 0); // Detenerse
                omnirobot_proxy->setSpeedBase(0, 0, 30); // Girar en el sitio
                QThread::sleep(1);  // Esperar un segundo mientras gira
            }
            else
            {
                // Si la distancia es suficientemente segura, continuar avanzando
                qInfo() << "Clear path, continuing...";
                omnirobot_proxy->setSpeedBase(1000, 0, 0); // Avanzar
            }

        }
        catch (const Ice::Exception& e)
        {
            std::cout << e.what() << std::endl;
        }

        // Evitar que el bucle consuma demasiado CPU (esperar un tiempo antes de la siguiente iteración)
        QThread::msleep(100);  // Esperar 100 ms entre cada ciclo de verificación
    }
}













//////////////////////////////////////////////////////////////////
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
