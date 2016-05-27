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

double absDouble(double input)
{
	(input <= 0)? input = -input: input = input;
	return input;
}


double convertAngle_Max_2PI(double angle)
{
	if(angle >= 0)
		return angle;
	else
		return 2*M_PI - absDouble(angle);
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

float angleToPoint(tf::Vector3 currentLocation, tf::Quaternion currentOrientation, tf::Vector3 lookedAtLocation)
{
	tf::Vector3 a = lookedAtLocation - currentLocation;
	//ROS_INFO_STREAM(a.getX() << " " << a.getY() << " " << a.getZ());

	tf::Matrix3x3 m(currentOrientation);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	a.setZ(0.0);
	tf::Vector3 currentPose_yAxis;
	currentPose_yAxis.setX(cos(yaw));
	currentPose_yAxis.setY(sin(yaw));
	currentPose_yAxis.setZ(0.0);
	float returnVal = a.angle(currentPose_yAxis);

	return returnVal;
}

double angleToPoint(tf::Vector3 currentLocation, tf::Vector3 lookedAtLocation)
{
	tf::Vector3 a = currentLocation;
	tf::Vector3 b = lookedAtLocation;
	tf::Vector3 c = b-a; c.setZ(0.0);
	tf::Vector3 x_Axis(1.0,0.0,0.0);
	double theta = c.angle(x_Axis);

	if(b.getY() >= a.getY())
		theta = theta;
	else
		theta = 2*M_PI - absDouble(theta);

	return theta;
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
	ros::init(argc,argv,"abc");   
	ros::NodeHandle nh;
	ros::Subscriber Lscan = nh.subscribe("scan",1,&scanMessageReceived);
	//ros::Subscriber mapmetadata = nh.subscribe("map",1, &getMapData);
	//ros::Subscriber amclPose = nh.subscribe("amcl_pose",1, &getAMCL);
	ros::Subscriber gazePose = nh.subscribe("gazebo_pose",1, &getPose);
	ros::Publisher abc = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

	ROS_INFO_STREAM("Run node...");
	Graph aGraph = Graph((std::string)"graph.txt");
	/*
	ceil_i = roundDouble((double) 1.0/resolution);
	ceil_j = roundDouble((double) 2.0/resolution);
	index_i = (ceil_i - ceil_i%10)/10;
	index_j = (ceil_j - ceil_j%10)/10;
	*/
	Node start;
	Node destination = Node(1, 4, 0);

	while(setup2 != 1)
	{
		ros::spinOnce();
	}
	ceil_i = roundDouble((double)pose.position.y/resolution);
	ceil_j = roundDouble((double)pose.position.x/resolution);
	index_i = (ceil_i - ceil_i%10)/10;
	index_j = (ceil_j - ceil_j%10)/10;
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
	double block_x = (path[index]->index_j*10 + 4.5)*resolution;
	double block_y = (path[index]->index_i*10 + 4.5)*resolution;
	int checkTurn = 0, initStore = 0, count = 0;
	double differenceStore;
	double angular, linear;

	int direction = checkDirectionOfNeighbor(*path[index], *path[index-1]);
	double angle = convertAngle_Max_2PI(findAngle(direction));
	int deviation = 0;
	int reachGoal = 0;
	double m_x, m_y;

	double rad = 0.1;
	ros::Rate loop_time(20);
	while(ros::ok())
	{
		geometry_msgs::Twist output;
		ros::spinOnce();

		tf::Matrix3x3 m(currentQua);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		yaw = convertAngle_Max_2PI(yaw);

		block_x = (path[index]->index_j*10 + 4)*resolution;
		block_y = (path[index]->index_i*10 + 4)*resolution;

		ceil_i = roundDouble((double)pose.position.y/resolution);
		ceil_j = roundDouble((double)pose.position.x/resolution);
		index_i = (ceil_i - ceil_i%10)/10;
		index_j = (ceil_j - ceil_j%10)/10;

		tf::Vector3 gotoPoint(block_x, block_y, 0.0);
		
		//ROS_INFO_STREAM(index_i << " " << index_j << " " << index);
		double theta = angleToPoint(currentPose, gotoPoint);
		double difference = absDouble(theta - yaw);

		//ROS_INFO_STREAM(block_x << " " << block_y << " " << pose.position.x << " " << pose.position.y);
		if(angleToPoint(currentPose, currentQua, gotoPoint) > 0.05)
		{
			if(initStore == 0)
			{
				output.angular.z = 0.5;
				output.linear.x = 0;
				differenceStore = angleToPoint(currentPose, currentQua, gotoPoint);
				if(count == 5) initStore = 1;
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
					if(differenceStore > angleToPoint(currentPose, currentQua, gotoPoint))
					{
						output.angular.z = 0.5;
						output.linear.x = 0;
					}
					else
					{
						output.angular.z = -0.5;
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
			initStore = 0; checkTurn = 0; count = 0;
			bool Object = checkObjectAhead(0.5);

			if(pose.position.x < block_x + rad && pose.position.x > block_x - rad && pose.position.y < block_y + rad && pose.position.y > block_y - rad)
			{
				index--;
				if(index == -1)
					break;
				continue;
			}

			if(Object)
			{
				if(index_i == path[index]->index_i && index_j == path[index]->index_j)
				{
					output.angular.z = 0;
					output.linear.x = 0;
					index--;
					if(index == -1)
						break;
				}
				else
				{
					ROS_INFO_STREAM(index_i << " " << index_j);
				}
			}
			else
			{
				output.angular.z = 0;
				output.linear.x = 0.7;
			}
		}

		/*
		if(checkAngle)
		{
			if(difference > 0.05)
			{
				ROS_INFO_STREAM("turn");
				if(theta - yaw > M_PI)
				{
					output.angular.z = -0.5;
					output.linear.x = 0;
				}
				else
				{
					output.angular.z = 0.5;
					output.linear.x = 0;
				}
			}
			else
			{
				checkAngle = 0;
			}
		}
		else
		{
			if(pose.position.x < block_x + rad && pose.position.x > block_x - rad && pose.position.y < block_y + rad && pose.position.y > block_y - rad)
			{
				ROS_INFO_STREAM("change");
				index--;
				if(index == -1)
					break;
				index_i = path[index]->index_i;
				index_j = path[index]->index_j;
				block_x = (index_j*10 + 4.5)*resolution;
				block_y = (index_i*10 + 4.5)*resolution;
				checkAngle = 1;
				continue;
			}
			else
			{
				output.angular.z = 0;
				output.linear.x = 0.7;
			}
		}

		//if robot enter other block
		/*
		if(index_i != path[index]->index_i || index_j != path[index]->index_j)
		{
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
		*/
		abc.publish(output);
		loop_time.sleep();
	}
	//mapmetadata.shutdown();
	
	return 0;
}