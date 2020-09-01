/*
 * SimulationController.h
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#ifndef SRC_SIMULATIONCONTROLLER_H_
#define SRC_SIMULATIONCONTROLLER_H_

#include "SimulationState.h"
#include "ArloDriver.h"
#include "NeuroControllerDriver.h"
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include "arlo_nn_controller/EvaluateDriver.h"
#include <string>
#include <iostream>

#define NUM_RAYS 32
#define NUM_SONARS 3
#define NUM_ACTUATORS 2 // Velocity linear and angular

using namespace std;

class SimulationController
{
public:
	SimulationController(double maxSTime = 300, int tRate = 1);
	virtual ~SimulationController();
	void setDrivers(ArloDriver *driver1, ArloDriver *driver2, ArloDriver *driver3);
	SimulationState startSimulation(ArloDriver *driver1, ArloDriver *driver2, ArloDriver *driver3, int maxtime);
	void checkSonarLeftValues1(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkSonarLeftValues2(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkSonarLeftValues3(const sensor_msgs::LaserScan::ConstPtr &msg);

	void checkSonarCenterValues1(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkSonarCenterValues2(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkSonarCenterValues3(const sensor_msgs::LaserScan::ConstPtr &msg);

	void checkSonarRightValues1(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkSonarRightValues2(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkSonarRightValues3(const sensor_msgs::LaserScan::ConstPtr &msg);

	void checkModelPosition(const nav_msgs::Odometry::ConstPtr &msg);
	void checkSimulationTime(const rosgraph_msgs::Clock::ConstPtr &msg);
	int getNumSensors();
	int getNumActuators();
	bool evaluateDriver(arlo_nn_controller::EvaluateDriver::Request &req,
						arlo_nn_controller::EvaluateDriver::Response &res);
	void checkBoxPosition(const gazebo_msgs::ModelStates::ConstPtr &msg);

	private: 
	double dist2Go(double x, double y);
	double distance(double x1, double y1, double x2, double y2);
	SimulationState arloState;
	double prev_x, prev_y;
	long int stuckCounter;
	bool stuck;
	string inputFile;
	string outputFile;
	double maxSimTime; /* Maximum time allowed for the robot to get the goal */
	double goalDistance;

	ArloDriver *aDriver1;
	ArloDriver *aDriver2;
	ArloDriver *aDriver3;

	int ticsRate;	 /* How often the driver is ask for a decision */
	double linear_1; /* Linear velocity to send to the robot */
	double linear_2; /* Linear velocity to send to the robot */
	double linear_3; /* Linear velocity to send to the robot */

	double angular_1; /* Angular velocity to send to the robot */
	double angular_2; /* Angular velocity to send to the robot */
	double angular_3; /* Angular velocity to send to the robot */

	double l_scale_; /* Factor to the scale the linear velocity value */
	double a_scale_; /* Factor to the scale the angular velocity value */
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_1;
	ros::Publisher vel_pub_2;
	ros::Publisher vel_pub_3;

	ros::Subscriber box_sub_;

	ros::Subscriber odom_sub_1;
	ros::Subscriber odom_sub_2;
	ros::Subscriber odom_sub_3;

	ros::Subscriber clock_sub_;
	ros::Subscriber sonar_l_sub_1;
	ros::Subscriber sonar_l_sub_2;
	ros::Subscriber sonar_l_sub_3;

	ros::Subscriber sonar_c_sub_1;
	ros::Subscriber sonar_c_sub_2;
	ros::Subscriber sonar_c_sub_3;

	ros::Subscriber sonar_r_sub_1;
	ros::Subscriber sonar_r_sub_2;
	ros::Subscriber sonar_r_sub_3;

	ros::ServiceServer service;
	vector<double> actuatorValues1;
	vector<double> actuatorValues2;
	vector<double> actuatorValues3;

	vector<double> sensorValues1;
	vector<double> sensorValues2;
	vector<double> sensorValues3;

	//double currentTime;
	//bool hasTimeRunOut;
	//bool finishLineCrossed;
};

#endif /* SRC_SIMULATIONCONTROLLER_H_ */
