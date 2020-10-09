/*
 * SimulationController.cpp
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#include "SimulationController.h"

SimulationController::SimulationController(double maxSTime, int tRate) : linear_1{0},
                                                                         linear_2{0},
                                                                         linear_3{0},
                                                                         angular_1{0},
                                                                         angular_2{0},
                                                                         angular_3{0},
                                                                         l_scale_{1.0},
                                                                         a_scale_{1.0},
                                                                         maxSimTime{maxSTime},
                                                                         ticsRate{tRate},
                                                                         actuatorValues1{NUM_ACTUATORS, 0.0},
                                                                         actuatorValues2{NUM_ACTUATORS, 0.0},
                                                                         actuatorValues3{NUM_ACTUATORS, 0.0}
{
   sensorValues1.resize(NUM_RAYS * NUM_SONARS);
   sensorValues2.resize(NUM_RAYS * NUM_SONARS);
   sensorValues3.resize(NUM_RAYS * NUM_SONARS);

   prev_x = 0;
   prev_y = 0;
   stuck = false;
   stuckCounter = 0;
   nh_.param("scale_angular", a_scale_, a_scale_);
   nh_.param("scale_linear", l_scale_, l_scale_);

   vel_pub_1 = nh_.advertise<geometry_msgs::Twist>("xolo1/cmd_vel", 1);
   vel_pub_2 = nh_.advertise<geometry_msgs::Twist>("xolo2/cmd_vel", 1);
   vel_pub_3 = nh_.advertise<geometry_msgs::Twist>("xolo3/cmd_vel", 1);


   clock_sub_ = nh_.subscribe("clock", 1000, &SimulationController::checkSimulationTime, this);

   box_sub_ = nh_.subscribe("/gazebo/model_states", 1000, &SimulationController::checkRobotsPosition, this);

   sonar_l_sub_1 = nh_.subscribe("xolo1/scan_left", 1000, &SimulationController::checkSonarLeftValues1, this);
   sonar_l_sub_2 = nh_.subscribe("xolo2/scan_left", 1000, &SimulationController::checkSonarLeftValues2, this);
   sonar_l_sub_3 = nh_.subscribe("xolo3/scan_left", 1000, &SimulationController::checkSonarLeftValues3, this);

   sonar_c_sub_1 = nh_.subscribe("xolo1/scan_center", 1000, &SimulationController::checkSonarCenterValues1, this);
   sonar_c_sub_2 = nh_.subscribe("xolo2/scan_center", 1000, &SimulationController::checkSonarCenterValues2, this);
   sonar_c_sub_3 = nh_.subscribe("xolo3/scan_center", 1000, &SimulationController::checkSonarCenterValues3, this);

   sonar_r_sub_1 = nh_.subscribe("xolo1/scan_right", 1000, &SimulationController::checkSonarRightValues1, this);
   sonar_r_sub_2 = nh_.subscribe("xolo2/scan_right", 1000, &SimulationController::checkSonarRightValues2, this);
   sonar_r_sub_3 = nh_.subscribe("xolo3/scan_right", 1000, &SimulationController::checkSonarRightValues3, this);

   service = nh_.advertiseService("evaluate_driver", &SimulationController::evaluateDriver, this);
}

SimulationController::~SimulationController()
{
}

void SimulationController::setDrivers(ArloDriver *driver1, ArloDriver *driver2, ArloDriver *driver3)
{
   aDriver1 = driver1;
   aDriver2 = driver2;
   aDriver3 = driver3;
}

bool SimulationController::evaluateDriver(arlo_nn_controller::EvaluateDriver::Request &req,
                                          arlo_nn_controller::EvaluateDriver::Response &res)
{
   //TODO: Revisar que Gazebo este corriendo

   string pesos1(req.weightsfile1);
   string pesos2(req.weightsfile2);
   string pesos3(req.weightsfile3);

   aDriver1->setParameters(pesos1.c_str());
   aDriver2->setParameters(pesos2.c_str());
   aDriver3->setParameters(pesos3.c_str());

   startSimulation(aDriver1, aDriver2, aDriver3, req.maxtime);

   res.time = arloState.finishTime;
   res.dist2go = arloState.distanceToGo;
   res.damage = arloState.robotDamage;
   res.energy = arloState.distanceTravelled;
   res.boxDistance = arloState.averageDistance;
   return true;
}

SimulationState SimulationController::startSimulation(ArloDriver *aDriver1, ArloDriver *aDriver2, ArloDriver *aDriver3, int maxtime)
{
   //string pesos(req.weightsfile);
   //aDriver->setParameters(pesos.c_str());

   puts("Starting the simulation of a new driver...");
   puts("---------------------------");

   std_srvs::Empty gazeboParams;
   ros::service::call("/gazebo/reset_simulation", gazeboParams);
   maxSimTime = maxtime;

   ros::Rate loop_rate(50); // Frecuencia Hz con la que le robot debe tomar una decisión.

   linear_1 = linear_2 = linear_3 = angular_1 = angular_2 = angular_3 = 0;

   arloState.resetState();
   stuckCounter = 0;

   while (ros::ok() && !arloState.hasTimeRunOut && !arloState.finishLineCrossed)
   {
      // Send sensor values to the driver and get its answer to move the robot.
      aDriver1->driveArlo(sensorValues1, actuatorValues1);
      aDriver2->driveArlo(sensorValues2, actuatorValues2);
      aDriver3->driveArlo(sensorValues3, actuatorValues3);

      //cout << "lineal= " << actuatorValues[0]
      //     << ", angular= " << actuatorValues[1] << "\n" << endl;

      linear_1 = actuatorValues1[0];
      linear_2 = actuatorValues2[0];
      linear_3 = actuatorValues3[0];

      angular_1 = actuatorValues1[1];
      angular_2 = actuatorValues2[1];
      angular_3 = actuatorValues3[1];

      // Set values in the twist object to send the actuator values to the robot in Gazebo.
      geometry_msgs::Twist twist1;
      geometry_msgs::Twist twist2;
      geometry_msgs::Twist twist3;

      twist1.angular.z = a_scale_ * angular_1;
      twist2.angular.z = a_scale_ * angular_2;
      twist3.angular.z = a_scale_ * angular_3;

      twist1.linear.x = l_scale_ * linear_1;
      twist2.linear.x = l_scale_ * linear_2;
      twist3.linear.x = l_scale_ * linear_3;

      vel_pub_1.publish(twist1); // Publish the event for the twist plugin.
      vel_pub_2.publish(twist2); // Publish the event for the twist plugin.
      vel_pub_3.publish(twist3); // Publish the event for the twist plugin.

      ros::spinOnce();
      loop_rate.sleep();
   }

   cout << "hasTimeRunOut= " << arloState.hasTimeRunOut << "\n";
   cout << "finishLineCrossed= " << arloState.finishLineCrossed << "\n";

   if (arloState.hasTimeRunOut == true)
   {
      arloState.finishTime = 2 * maxSimTime;
      //arloState.distanceToGo = goalDistance - arloState.currentPosition;
      cout << "currentPosition= " << arloState.currentPosition << "\n";
      if (arloState.stuck == true)
      {
         cout << " ---->>> ATASCADO  <<<-----" << endl;
         cout << " \t Counter = " << stuckCounter << endl;
      }
   }
   else
   { // The robot reached the goal.
      arloState.finishTime = arloState.currentTime;
      //arloState.distanceToGo = 0.0;
      arloState.robotEnergy = 100;
      cout << "finishTime= " << arloState.finishTime << "\n";
   }

   cout << "x = " << arloState.position[0] << ", y = " << arloState.position[1] << endl;
   cout << "Average Distance= " << arloState.averageDistance << endl;
   cout << "gas= " << arloState.distanceTravelled << endl;

   //	res.time = arloState.finishTime;
   //   res.dist2go = arloState.distanceToGo;
   //	res.damage = arloState.robotDamage ;
   //	res.energy = arloState.distanceTravelled;

   ros::service::call("/gazebo/reset_simulation", gazeboParams);
   std::cout << "Bye" << std::endl;

   return arloState;
}

void SimulationController::checkSonarLeftValues1(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges left= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues1[i + 0 * NUM_RAYS] = msg->ranges[i]; // 0 para el sensor izq.
   }
}

void SimulationController::checkSonarLeftValues2(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges left= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues2[i + 0 * NUM_RAYS] = msg->ranges[i]; // 0 para el sensor izq.
   }
}

void SimulationController::checkSonarLeftValues3(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges left= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues3[i + 0 * NUM_RAYS] = msg->ranges[i]; // 0 para el sensor izq.
   }
}

void SimulationController::checkSonarCenterValues1(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges center= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues1[i + 1 * NUM_RAYS] = msg->ranges[i]; // 1 para el sensor central.
   }
}

void SimulationController::checkSonarCenterValues2(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges center= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues2[i + 1 * NUM_RAYS] = msg->ranges[i]; // 1 para el sensor central.
   }
}

void SimulationController::checkSonarCenterValues3(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges center= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues3[i + 1 * NUM_RAYS] = msg->ranges[i]; // 1 para el sensor central.
   }
}

void SimulationController::checkSonarRightValues1(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges right= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues1[i + 2 * NUM_RAYS] = msg->ranges[i]; // 2 para el sensor der.
   }
}

void SimulationController::checkSonarRightValues2(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges right= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues2[i + 2 * NUM_RAYS] = msg->ranges[i]; // 2 para el sensor der.
   }
}

void SimulationController::checkSonarRightValues3(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   //ROS_INFO("Tamano ranges right= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues3[i + 2 * NUM_RAYS] = msg->ranges[i]; // 2 para el sensor der.
   }
}

double SimulationController::dist2Go(double x, double y)
{
   double distToGo;
   if (y < 0.8 && x < 4.7)
   { // Va en la primera recta
      distToGo = 18 - x;
   }
   else if (y < 6.61)
   { // Va en la segunda recta
      distToGo = 18 - (5.25 + y);
   }
   else
   { // Va en la recta final
      if (x > 0.0)
         distToGo = x;
      else
         distToGo = 0.0;
   }
}

double SimulationController::distance(double x1, double y1, double x2, double y2)
{
   double sum = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
   return sqrt(sum);
}

void SimulationController::checkRobotsPosition(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
   arloState.iteration++;

   if (arloState.iteration < 500)
   {
      // Do nothing
      return;
   }
   arloState.iteration = 0;

   // Get position of the 3 robots
    double x1 = msg->pose[1].position.x;
    double y1 = msg->pose[1].position.y;
    double x2 = msg->pose[2].position.x;
    double y2 = msg->pose[2].position.y;
    double x3 = msg->pose[3].position.x;
    double y3 = msg->pose[3].position.y;
   // Get the distance between robots
   // Robot 1 and 2
   double d1 = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
   // Robot 1 and 3
   double d2 = sqrt(pow(x3-x1, 2) + pow(y3-y1, 2));
   // Robot 2 and 3
   double d3 = sqrt(pow(x3-x2, 2) + pow(y3-y2, 2));

   arloState.averageDistance += (d1 + d2 + d3) /3;

   // ROS_INFO("Name: %s", msg->name[5].c_str());
   if (x1 >= 12 && x2 >= 12 && x3 >= 12)
   {
      arloState.finishLineCrossed = true;
   }
}

void SimulationController::checkModelPosition(const nav_msgs::Odometry::ConstPtr &msg)
{
   //ROS_INFO("Seq: [%d]", msg->header.seq);
   //ROS_INFO("Position-> x: [%f]", msg->pose.pose.position.x);
   //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
   //		msg->twist.twist.linear.x,
   //		msg->twist.twist.angular.z);

   // arloState.currentPosition = msg->pose.pose.position.x;
   // if (arloState.currentPosition >= goalDistance)   // Esta es la distancia del pasillo en Gazebo.
   // 	arloState.finishLineCrossed = true;

   double distanceBefore = arloState.distanceTravelled;
   arloState.distanceTravelled += distance(prev_x, prev_y, msg->pose.pose.position.x, msg->pose.pose.position.y);

   if (abs(distanceBefore - arloState.distanceTravelled) < 0.01)
   {
      stuckCounter++;
      //cout << "Stuck counter: " << stuckCounter << "\n";
      if (stuckCounter > 80)
      {
         arloState.stuck = true;
         arloState.hasTimeRunOut = true;
      }
   }
   else
      stuckCounter = 0;

   prev_x = msg->pose.pose.position.x;
   prev_y = msg->pose.pose.position.y;

   arloState.currentPosition = msg->pose.pose.position.x;
   arloState.position[0] = msg->pose.pose.position.x;
   arloState.position[1] = msg->pose.pose.position.y;
   arloState.distanceToGo = dist2Go(msg->pose.pose.position.x, msg->pose.pose.position.y);
   if (arloState.distanceToGo <= 0.0) // Esta es la distancia del pasillo en Gazebo.
      arloState.finishLineCrossed = true;
}

void SimulationController::checkSimulationTime(const rosgraph_msgs::Clock::ConstPtr &msg)
{
   arloState.currentTime = msg->clock.toSec();
   //std::cout << "Tiempo simulacion: " <<  msg->clock << std::endl;

   if (arloState.currentTime >= maxSimTime)
   {
      arloState.hasTimeRunOut = true;
   }
}

int SimulationController::getNumSensors()
{
   return NUM_RAYS * NUM_SONARS;
}

int SimulationController::getNumActuators()
{
   return NUM_ACTUATORS;
}
