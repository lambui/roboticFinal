#include <ros/ros.h> 
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
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
#include "node_graph.h"

int done = 0;

void getMapData(const nav_msgs::OccupancyGrid msg)
{
	ROS_INFO_STREAM(msg.info.resolution);
	ROS_INFO_STREAM(msg.info.width);
	ROS_INFO_STREAM(msg.info.height);

	int row = msg.info.height;
	int col = msg.info.width;

	std::ofstream myfile, myfile2, myfile3;
	myfile.open("rawData.txt");

	std::vector<std::vector<int> > matrix;
	
	int count = 0;
	for(int i = 0; i < row; i++)
	{
		if(i%10 == 0)
		{
			std::vector<int> temp;
			matrix.push_back(temp);
		}
		for(int j = 0; j < col; j++)
		{
			if(i%10 == 0 && j%10 == 0)
			{
				matrix.back().push_back(count);
				count++;
			}
			myfile << (int)msg.data[j + col*i] << " 	";
		}
		myfile << "\n";
	}
	ROS_INFO_STREAM("done");
	myfile.close();

	int block_size = 10;
	myfile2.open("nodeMap.txt");
	for(int i = 0; i < matrix.size(); i++)
	{
		for(int j = 0; j < matrix[i].size(); j++)
		{
			int occupied = 0;
			for(int a = 0; a < block_size; a++)
			{
				for(int b = 0; b < block_size; b++)
				{
					int row_index = i*block_size + a;
					int col_index = j*block_size + b;
					if(col_index + col*row_index < row*col)
						if((int)msg.data[col_index + col*row_index] != 0)
						{
							occupied = 1;
							break;
						}
					else
					{
						break;
					}
				}
				if(occupied)
					break;
			}
			matrix[i][j] = occupied;
			myfile2 << matrix[i][j] << "	";
		}
		myfile2 << "\n";
	}
	ROS_INFO_STREAM("done");
	myfile2.close();

	std::vector<std::vector<Node> > matrix2;
	for(int i = 0; i < matrix.size(); i++)
	{
		std::vector<Node> temp;
		matrix2.push_back(temp);
		for(int j = 0; j < matrix[i].size(); j++)
		{
			Node aNode = Node(i,j,matrix[i][j]);
			matrix2.back().push_back(aNode);
		}
	}

	for(int i = 0; i < matrix2.size(); i++)
	{
		for(int j = 0; j < matrix2[i].size(); j++)
		{
			matrix2[i][j].checkNeighbors(matrix2, matrix2.size(), matrix2[i].size());
		}
	}

	myfile3.open("graph.txt");
	for(int i = 0; i < matrix2.size(); i++)
	{
		for(int j = 0; j < matrix2[i].size(); j++)
		{
			if(matrix2[i][j].occupied == 0)
			{
				std::string output = matrix2[i][j].printCoors() + "\n";
				myfile3 << output;
			}
		}
	}
	myfile3.close();
	done = 1;
}

int main(int argc,char **argv) 
{
	ros::init(argc,argv,"mapInfo");   
	ros::NodeHandle nh;
	ros::Subscriber mapmetadata = nh.subscribe("map",1, &getMapData);
	ROS_INFO_STREAM("Run node...");
	while(done != 1)
	{
		ros::spinOnce();
	}
	return 0;
}