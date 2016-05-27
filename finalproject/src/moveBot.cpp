#include <ros/ros.h> 
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <climits>
#include <algorithm>
#include "dijktra.h"

/*
map's coordinate frame:
x <------------------------	(0,0)
								|
								|
								|
								|
								|
								|
								|
								|
								|
								|
								|
								V
								y


coordinate frame of graph on map:
index_j <---------------------------(0,0)
										|
										|
										|
										|
										|
										|
										|
										|
										|
										|
										|
										V
									index_i		
*/

int setup1 = 0; int setup2 = 0;
double resolution = 0.05;
geometry_msgs::Pose pose; 
int ceil_i, ceil_j;
int index_i, index_j;
int index_i2, index_j2;
int calibrated = 1;
tf::Vector3 currentPose;
tf::Quaternion currentQua;
float diagonal;

double to2pi(double input)
{
	while(input < 0)
		input += 2*M_PI;

	while(input >= 2*M_PI)
		input -= 2*M_PI;

	return input;
}

double absDouble(double input)
{
	(input <= 0)? input = -input: input = input;
	return input;
}

int roundDouble(double input)
{
	double flo = floor(input);
	double cei = ceil(input);
	if(absDouble(input-flo) <= absDouble(input-cei))
		return (int) flo;
	else
		return (int) cei;
}

double findAngle(int direction)
{
	switch(direction)
	{
		case 0: return -M_PI/2; //up
		case 1: return 0; //right
		case 2: return M_PI/2; //down
		case 3: return M_PI; //left
	}
}

double angleToPoint(tf::Vector3 currentLocation, tf::Quaternion currentOrientation, tf::Vector3 lookedAtLocation)
{
	tf::Vector3 a = lookedAtLocation - currentLocation; a.setZ(0.0);

	tf::Matrix3x3 m(currentOrientation);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	tf::Vector3 currentPose_xAxis;
	currentPose_xAxis.setX(1.0);
	currentPose_xAxis.setY(0.0);
	currentPose_xAxis.setZ(0.0);
	double angle = a.angle(currentPose_xAxis);

	yaw = to2pi(yaw);
	if(a.getY() < 0) angle = 2*M_PI - angle;
	
	double returnVal = absDouble(yaw - angle);
	if(returnVal > M_PI)
		returnVal = 2*M_PI - returnVal;

	ROS_INFO_STREAM(yaw << " " << angle << " " << returnVal);

	return returnVal;
}

double angleToNextNode(tf::Quaternion currentOrientation, double angle)
{
	tf::Matrix3x3 m(currentOrientation);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	
	yaw = to2pi(yaw);
	angle = to2pi(angle);

	double returnVal = absDouble(yaw - angle);
	if(returnVal > M_PI)
		returnVal = 2*M_PI - returnVal;

	ROS_INFO_STREAM(yaw << " " << angle << " " << returnVal);

	return returnVal;
}

float angle_min, angle_max, angle_increment;
std::vector<float> ranges;
void scanMessageReceived(const sensor_msgs::LaserScan msg) 
{
	ranges.clear();
	angle_min = msg.angle_min;
	angle_max = msg.angle_max;
	angle_increment = msg.angle_increment;
	ranges = msg.ranges;
} 

bool checkObjectAhead(float distance)
{
	bool Object = false; //assume no object ahead
	int degree = 60;
	for(int i = 0; i < ranges.size(); i++) //4 indexes per angle, so 60degree is 240
	{
		if(i >= (90-degree)*4 && i <= (90+degree)*4) //check only from 'degree' degree to the left to 'degree' degree to the right
			if(ranges[i] < distance)
			{
				Object = true;
				break;				
			}
	}
	return Object;
}

void getMapData(const nav_msgs::OccupancyGrid msg)
{
	resolution = msg.info.resolution;
	setup1 = 1;
}

void getAMCL(const geometry_msgs::PoseWithCovarianceStamped msg) 
{
	pose = msg.pose.pose;
	currentPose.setX(pose.position.x);
	currentPose.setY(pose.position.y);
	currentPose.setZ(pose.position.z);

	currentQua.setX(pose.orientation.x);
	currentQua.setY(pose.orientation.y);
	currentQua.setZ(pose.orientation.z);
	currentQua.setW(pose.orientation.w);
	currentQua.normalize();

	diagonal = msg.pose.covariance[0]+	msg.pose.covariance[7]+	msg.pose.covariance[14]+	msg.pose.covariance[21]+	msg.pose.covariance[28]+	msg.pose.covariance[35];
	setup2 = 1;
}
void getPose(const geometry_msgs::Pose msg)
{
	pose = msg;
	currentPose.setX(msg.position.x);
	currentPose.setY(msg.position.y);
	currentPose.setZ(msg.position.z);

	currentQua.setX(msg.orientation.x);
	currentQua.setY(msg.orientation.y);
	currentQua.setZ(msg.orientation.z);
	currentQua.setW(msg.orientation.w);

	setup2 = 1;
}

int main(int argc,char **argv) 
{
	ros::init(argc,argv,"moveBot");   
	ros::NodeHandle nh;
	ros::Subscriber Lscan = nh.subscribe("scan",1,&scanMessageReceived);
	//ros::Subscriber mapmetadata = nh.subscribe("map",1, &getMapData);
	ros::Subscriber amclPose = nh.subscribe("amcl_pose",1, &getAMCL);
	//ros::Subscriber gazePose = nh.subscribe("gazebo_pose",1, &getPose);
	ros::Publisher abc = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

	ROS_INFO_STREAM("Run node...");
	Graph aGraph = Graph((std::string)"graph.txt");

	Node start;
	Node destination = Node(1, 5, 0);
	//Node destination = Node(30, 30, 0);


	while(setup2 != 1)
	{
		ros::spinOnce();
	}
	ceil_i = roundDouble((double)pose.position.y/resolution);
	ceil_j = roundDouble((double)pose.position.x/resolution);
	index_i = (ceil_i - ceil_i%10)/10;
	index_j = (ceil_j - ceil_j%10)/10;
	ROS_INFO_STREAM(index_i << " " << index_j);

	//ROS_INFO_STREAM(resolution);
	start.index_i = index_i;
	start.index_j = index_j;

	std::pair<std::vector<Node*>, std::vector<Node*> > result;
	result = dijkstra(aGraph, start, destination);
	std::vector<Node*> path;

	if(result.second[0]->index_i != -1)
	{
		ROS_INFO_STREAM(result.first.back()->index_i << " " << result.first.back()->index_j);
		path.push_back(aGraph.vertexList[aGraph.searchVertex(*result.first.back())]);
		int index = checkNode(result.first, *result.second.back());
		while(true)
		{
			ROS_INFO_STREAM(result.first[index]->index_i << " " << result.first[index]->index_j);
			path.push_back(aGraph.vertexList[aGraph.searchVertex(*result.first[index])]);
			if(index == 0)
				break;
			else
				index = checkNode(result.first, *result.second[index]);
		}
	}
	
	int index = path.size()-1;
	int direction = checkDirectionOfNeighbor(*path[index], *path[index-1]);
	int direction2 = direction;
	double angle = findAngle(direction);
	index_i2 = index_i; index_j2 = index_j;
	int deviation = 0;
	int objectDeviation = 0, first = 0; int o_i, o_j;
	int reachGoal = 0;
	double m_x, m_y;

	int checkTurn = 0, initStore = 0, count = 0;
	double differenceStore, initDif;
	double angular, linear;

	double turn_speed = 0.2;
	double run_speed = 0.5;

	int stopCount = 0;
	int delay = 0;

	ros::Rate loop_time(50);
	while(ros::ok())
	{
		geometry_msgs::Twist output;
		ros::spinOnce();

		tf::Matrix3x3 m(currentQua);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		//ROS_INFO_STREAM(yaw);
		ceil_i = roundDouble((double)pose.position.y/resolution);
		ceil_j = roundDouble((double)pose.position.x/resolution);
		index_i = (ceil_i - ceil_i%10)/10;
		index_j = (ceil_j - ceil_j%10)/10;

		ROS_INFO_STREAM(index_i << " " << index_j << " " << index);

		//if robot enter other block
		//bool Object = checkObjectAhead(0.3);

		if(index_i != path[index]->index_i || index_j != path[index]->index_j)
		{
			initStore = 0; checkTurn = 0; count = 0; differenceStore = 0; initDif = 0;
			//check if robot stay on track
			if(index_i == path[index-1]->index_i && index_j == path[index-1]->index_j)
			{
				index = checkNode(path, index_i, index_j);
				if(index != 0 && index != -1)
					direction = checkDirectionOfNeighbor(*path[index], *path[index-1]);
				else
					reachGoal = 1;
				angle = findAngle(direction);
			}
			else //check for deviation
			{
				deviation = 1;
			}
		}

		if(reachGoal == 0)
		{
			if(deviation == 0)
			{
				double difference = angleToNextNode(currentQua, angle);
				if(difference > 0.07)
				{
					delay = 0;
					if(initStore == 0)
					{
						output.angular.z = turn_speed;
						output.linear.x = 0;
						differenceStore = difference;
						if(count == 30)
						{
							initStore = 1;
							count = 0;
						}
						count++;
					}
					else
					{
						if(count == 20)
						{
							if(checkTurn == 1) //done checking turn
							{
								output.angular.z = angular;
								output.linear.x = linear;
							}
							else //check if the turn is the right one
							{
								if(differenceStore > angleToNextNode(currentQua, angle))
								{
									output.angular.z = turn_speed;
									output.linear.x = 0;
								}
								else
								{
									output.angular.z = -turn_speed;
									output.linear.x = 0;
								}
								angular = output.angular.z;
								linear = output.linear.z;
								checkTurn = 1;
							}
						}
						else
						{
							output.angular.z = turn_speed;
							output.linear.x = 0;
							count++;
						}
					}
				}
				else
				{
					initStore = 0; checkTurn = 0; count = 0; differenceStore = 0; initDif = 0;
					if(delay == 20)
					{
						output.linear.x = run_speed;
						output.angular.z = 0;
					}
					else
					{
						output.linear.x = 0;
						output.angular.z = 0;
						delay++;
					}
				}
			}
			else
			{
				ROS_INFO_STREAM("DEVIATE");
				int temp = checkNode(path, index_i, index_j);
				if(temp != -1)
				{
					deviation = 0;
					index = temp+1;
					initStore = 0; checkTurn = 0; count = 0; differenceStore = 0; initDif = 0;
				}
				m_x = (path[index-1]->index_j*10+5)*resolution;
				m_y = (path[index-1]->index_i*10+5)*resolution;
				tf::Vector3 goTo;
				goTo.setX(m_x);
				goTo.setY(m_y);
				goTo.setZ(0.0);
				double difference = angleToPoint(currentPose, currentQua, goTo);
				if(difference > 0.1)
				{
					delay = 0;
					if(initStore == 0)
					{
						output.angular.z = 0.5;
						output.linear.x = 0;
						differenceStore = difference;
						if(count == 0) initDif = difference;
						if(count == 20)
						{
							initStore = 1;
							count = 0;
						}
						count++;
					}
					else
					{
						if(checkTurn) //done checking turn
						{
							output.angular.z = angular;
							output.linear.x = linear;
						}
						else //check if the turn is the right one
						{
							if(angleToPoint(currentPose, currentQua, goTo) > initDif)
							{
								output.angular.z = -0.5;
								output.linear.x = 0;
							}
							else
							{
								output.angular.z = 0.5;
								output.linear.x = 0;
							}
							angular = output.angular.z;
							linear = output.linear.z;
							checkTurn = 1;
						}
					}
				}
				else
				{			
					initStore = 0; checkTurn = 0; count = 0; differenceStore = 0; initDif = 0;
					if(delay == 20)
					{
						output.linear.x = run_speed;
						output.angular.z = 0;
					}
					else
					{
						output.linear.x = 0;
						output.angular.z = 0;
						delay++;
					}
				}
			}
		}
		else
		{
			output.angular.z = 0;
			output.linear.x = 0;
			abc.publish(output);
			if(stopCount == 10) break;
			stopCount++;
		}
		abc.publish(output);
		loop_time.sleep();
	}
	//mapmetadata.shutdown();
	
	return 0;
}