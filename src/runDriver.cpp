/*
 * startNode.cpp
 *
 *  Created on: Apr 15, 2020
 *      Author: antonio
 */

#include "SimulationController.h"
#include "ArloDriver.h"
#include "NeuroControllerDriver.h"
#include <ros/ros.h>
#include <vector>
#include <utility>

using namespace std;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "neurocontroller_node");

   if (argc < 3)
   {
      cout << "\nUsage: " << argv[0] << " <weightsfile>\n"
           << endl;
      return 0;
   }

   string pesos1(argv[1]);
   string pesos2(argv[2]);
   string pesos3(argv[3]);

   SimulationController sim;

   int numRays = sim.getNumSensors();
   int numActuators = sim.getNumActuators();

   vector<pair<double, double>> outputRanges;
   outputRanges.push_back(make_pair(-0.25, 2));
   outputRanges.push_back(make_pair(-0.5, 0.5));

   ArloDriver *driver1 = new NeuroControllerDriver(numRays, numActuators, outputRanges);
   ArloDriver *driver2 = new NeuroControllerDriver(numRays, numActuators, outputRanges);
   ArloDriver *driver3 = new NeuroControllerDriver(numRays, numActuators, outputRanges);

   driver1->setParameters(pesos1.c_str()); // Carga el archivo de pesos.
   driver2->setParameters(pesos2.c_str()); // Carga el archivo de pesos.
   driver3->setParameters(pesos3.c_str()); // Carga el archivo de pesos.

   sim.startSimulation(driver1, driver2, driver3, 50);

   ROS_INFO("The simulation has ended.");

   return (0);
}
