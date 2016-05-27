#ifndef DIJKTRA_H
#define DIJKTRA_H

#include <math.h>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <climits>
#include <algorithm>
#include "node_graph.h"

void merge(std::vector<Node*> &list, std::vector<int> &cost, std::vector<Node*> &backPointer, int p, int q, int r)
{
	std::vector<int> cost_temp;
	std::vector<Node**> list_temp;
	std::vector<Node**> backPointer_temp;
	int i = 0;
	int j = 0;
	for(int k = p; k <= r; k++)
	{
		if(cost[k+j] <= cost[k+i])
		{
			if(k+j > r) break;
			cost_temp.push_back(cost[k+j]);
			list_temp.push_back(&list[k+j]);
			backPointer_temp.push_back(&backPointer[k+j]);
			j++;
		}
		else
		{
			cost_temp.push_back(cost[k+i]);
			list_temp.push_back(&list[k+i]);
			backPointer_temp.push_back(&backPointer[k+i]);
			i++;
		}
	}

	for(int a = 0; a < cost_temp.size(); a++)
	{
		cost[p+a] = cost_temp[a];
		list[p+a] = *list_temp[a];
		backPointer[p+a] = *backPointer_temp[a];
	}
}

void mergeSort(std::vector<Node*> &list, std::vector<int> &cost, std::vector<Node*> &backPointer, int i, int j)
{
	if(i < j)
	{
		int q = floor((i+j)/2);
		mergeSort(list, cost, backPointer,i,q);
		mergeSort(list, cost, backPointer,q+1,j);
		merge(list, cost, backPointer,i,q,j);
	}
}

void selectionSort(std::vector<Node*> &list, std::vector<int> &cost, std::vector<Node*> &backPointer)
{
	int counter = 0;
	while(counter < cost.size())
	{
		int index = counter;
		int min = cost[index];
		for(int i = counter; i < cost.size(); i++)
		{
			if(min > cost[i])
			{
				min = cost[i];
				index = i;
			}
		}
		std::swap(list[counter], list[index]);
		std::swap(cost[counter], cost[index]);
		std::swap(backPointer[counter], backPointer[index]);
		counter++;
	}
}

std::pair<std::vector<Node*>, std::vector<Node*> > dijkstra(Graph &graph, Node start, Node end)
{
	std::pair<std::vector<Node*>, std::vector<Node*> > returnVal;
	std::vector<Node*> open;
	std::vector<Node*> close;
	std::vector<Node*> backPointer, backPointer2;
	std::vector<int> cost;
	for(int i = 0; i < graph.vertexList.size(); i++)
	{
		open.push_back(graph.vertexList[i]);
		cost.push_back(INT_MAX);
		backPointer.push_back(NULL);
	}
	int start_i = graph.searchVertex(start);
	cost[start_i] = 0;
	backPointer[start_i] = graph.vertexList[graph.searchVertex(*open[start_i])]; //parent is itself if it the start point

	int cycle = 3;
	int counter = 0;
	while(open.size() > 0)
	{
		selectionSort(open, cost, backPointer);
		//mergeSort(open, cost, backPointer, 0, cost.size()-1);
		close.push_back(open[0]);
		//std::cout << open[0]->index_i << " " << open[0]->index_j << std::endl;
		backPointer2.push_back(backPointer[0]);
		if(close.back()->index_i == end.index_i && close.back()->index_j == end.index_j)
		{
			returnVal = std::make_pair(close, backPointer2);
			return returnVal;
		}
		else
		{
			//up
			if(close.back()->up != NULL)
			{
				int index = checkNode(open, *close.back()->up);
				if(index != -1) //if child node in open
				{
					int costFromNode = cost[0] + graph.edgeCost[graph.searchEdge(*close.back(), *close.back()->up)];
					if(cost[index] > costFromNode)
					{
						cost[index] = costFromNode;
						backPointer[index] = graph.vertexList[graph.searchVertex(*close.back())];
					}
				}
			}

			//right
			if(close.back()->right != NULL)
			{
				int index = checkNode(open, *close.back()->right);
				if(index != -1)
				{
					int costFromNode = cost[0] + graph.edgeCost[graph.searchEdge(*close.back(), *close.back()->right)];
					if(cost[index] > costFromNode)
					{
						cost[index] = costFromNode;
						backPointer[index] = graph.vertexList[graph.searchVertex(*close.back())];
					}
				}
			}

			//down
			if(close.back()->down != NULL)
			{
				int index = checkNode(open, *close.back()->down);
				if(index != -1)
				{
					int costFromNode = cost[0] + graph.edgeCost[graph.searchEdge(*close.back(), *close.back()->down)];
					if(cost[index] > costFromNode)
					{
						cost[index] = costFromNode;
						backPointer[index] = graph.vertexList[graph.searchVertex(*close.back())];
					}
				}
			}

			//left
			if(close.back()->left != NULL)
			{
				int index = checkNode(open, *close.back()->left);
				if(index != -1)
				{
					int costFromNode = cost[0] + graph.edgeCost[graph.searchEdge(*close.back(), *close.back()->left)];
					if(cost[index] > costFromNode)
					{
						cost[index] = costFromNode;
						backPointer[index] = graph.vertexList[graph.searchVertex(*close.back())];
					}
				}
			}
		}
		backPointer.erase(backPointer.begin());
		open.erase(open.begin());
		cost.erase(cost.begin());
		//std::cout << count(cost, 1) << std::endl;
		//std::cout << open.size() << " " << cost.size() << " " << backPointer.size() << "\n";
	}

	backPointer2.clear();
	backPointer2.push_back(new Node()); //did not find node, so return the vector w first element having index_i = index_j = -1
	returnVal = std::make_pair(close, backPointer2);
	return returnVal;
}

#endif