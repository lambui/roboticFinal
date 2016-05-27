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
#include "dijktra.h"

/*
Graph's coordinate frame:
(0,0)--------------------------> index_j
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

int main(int argc,char **argv) 
{
	ros::init(argc,argv,"findPath");   
	ros::NodeHandle nh;
	//ros::Subscriber gazePose = nh.subscribe("gazebo_pose",1, &getPose);

	ROS_INFO_STREAM("Run node...");
	Graph aGraph = Graph((std::string)"graph.txt");

	int s_index_i, s_index_j;
	int e_index_i, e_index_j;
	std::cout << "Input start node's index_i: ";
	std::cin >> s_index_i;
	std::cout << "Input start node's index_j: ";
	std::cin >> s_index_j;
	std::cout << "Input end node's index_i: ";
	std::cin >> e_index_i;
	std::cout << "Input end node's index_j: ";
	std::cin >> e_index_j;

	/*
	while(setup != 1)
	{
		ros::spinOnce();
	}
	*/

	Node destination = Node(e_index_i, e_index_j, 0);
	Node start = Node(s_index_i,s_index_j,0);

	std::cout << "Start node:	( " << start.index_i << ", " << start.index_j << " )\n";
	std::cout << "End node:	( " << destination.index_i << ", " << destination.index_j << " )\n";
	std::cout<<std::endl;

	std::pair<std::vector<Node*>, std::vector<Node*> > result;
	result = dijkstra(aGraph, start, destination);
	std::vector<Node*> path;

	if(result.second[0]->index_i != -1)
	{
		path.push_back(aGraph.vertexList[aGraph.searchVertex(*result.first.back())]);
		int index = checkNode(result.first, *result.second.back());
		while(true)
		{
			path.push_back(aGraph.vertexList[aGraph.searchVertex(*result.first[index])]);
			if(index == 0)
				break;
			else
				index = checkNode(result.first, *result.second[index]);
		}
	}

	std::cout<<"Path stack: \n";
	for(int a = 0; a < path.size(); a++)
	{
		if(a == 0) std::cout << "	End----> ( " << path[a]->index_i << ", " << path[a]->index_j << " )\n";
		else if(a == path.size()-1) std::cout << "	Start--> ( " << path[a]->index_i << ", " << path[a]->index_j << " )\n";
		else std::cout << "		 ( " << path[a]->index_i << ", " << path[a]->index_j << " )\n";
	}
	std::cout<<std::endl;

	int min_i = path[0]->index_i;
	int max_i = path[0]->index_i;
	int min_j = path[0]->index_j;
	int max_j = path[0]->index_j;
	for(int a = 1; a < path.size(); a++)
	{
		if(min_i > path[a]->index_i)
			min_i = path[a]->index_i;
		if(max_i < path[a]->index_i)
			max_i = path[a]->index_i;
		if(min_j > path[a]->index_j)
			min_j = path[a]->index_j;
		if(max_j < path[a]->index_j)
			max_j = path[a]->index_j;
	}

	char matrix[2*(max_i - min_i + 1)][2*(max_j - min_j + 1)];
	for(int a = 0; a < 2*(max_i - min_i + 1); a++)
	{
		for(int b = 0; b < 2*(max_j - min_j + 1); b++)
		{
			matrix[a][b] = ' ';
		}
	}

	for(int a = 0; a < path.size(); a++)
	{
		int row = 2*(path[a]->index_i - min_i);
		int col = 2*(path[a]->index_j - min_j);
		matrix[row][col] = 'o';
		if(a == 0)
			matrix[row][col] = 'G';
		if(a == path.size()-1)
			matrix[row][col] = 'S';
		if(a != path.size()-1)
		{
			switch(checkDirectionOfNeighbor(*path[a], *path[a+1]))
			{
				case 0: matrix[row-1][col] = '|'; break;
				case 1: matrix[row][col+1] = '-'; break;
				case 2: matrix[row+1][col] = '|'; break;
				case 3: matrix[row][col-1] = '-'; break;
			}
		}
	}
	
	//revert horizontally to match with actual map
	for(int a = 0; a < 2*(max_i - min_i + 1); a++)
	{
		char temp[2*(max_j - min_j + 1)];
		int len = 2*(max_j - min_j + 1);
		for(int b = 0; b < 2*(max_j - min_j + 1); b++)
		{
			//temp[(len+b-2)%len] = matrix[a][b]; //shift left by 2
			temp[len-1-b] = matrix[a][b]; //revert
		}

		for(int b = 0; b < 2*(max_j - min_j + 1); b++)
		{
			matrix[a][b] = temp[b];
		}
	}

	//display
	std::cout<<"Visualized path: \n";
	for(int a = 0; a < 2*(max_i - min_i + 1); a++)
	{
		for(int b = 0; b < 2*(max_j - min_j + 1); b++)
		{
			std::cout << matrix[a][b];
		}
		std::cout << std::endl;
	}
}