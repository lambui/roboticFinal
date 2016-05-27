#ifndef NODE_GRAPH_H
#define NODE_GRAPH_H

#include <math.h>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <climits>
#include <algorithm>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

class Node
{
	public:
		int index_i;
		int index_j;
		Node *left;
		Node *right;
		Node *up;
		Node *down;
		int occupied;

	Node()
	{
		index_i = -1;
		index_j = -1;
		left = NULL;
		right = NULL;
		up = NULL;
		down = NULL;
		occupied = 0;
	}
	Node(int i, int j, int state)
	{
		index_i = i;
		index_j = j;
		left = NULL;
		right = NULL;
		up = NULL;
		down = NULL;
		occupied = state;
	}

	void checkNeighbors(std::vector<std::vector<Node> > &matrix, int row, int col)
	{
		int i = index_i;
		int j = index_j;
		if(i == 0 && j == 0)
		{
			if(matrix[i][j+1].occupied == 0) right = &matrix[i][j+1];
			if(matrix[i+1][j].occupied == 0) down = &matrix[i+1][j];
		}
		else if(i == 0 && j == col-1)
		{
			if(matrix[i][j-1].occupied == 0) left = &matrix[i][j-1];
			if(matrix[i+1][j].occupied == 0) down = &matrix[i+1][j];
		}
		else if(i == row-1 && j == 0)
		{
			if(matrix[i-1][j].occupied == 0) up = &matrix[i-1][j];
			if(matrix[i][j+1].occupied == 0) right = &matrix[i][j+1];
		}
		else if(i == row-1 && j == col-1)
		{
			if(matrix[i][j-1].occupied == 0) left = &matrix[i][j-1];
			if(matrix[i-1][j].occupied == 0) up = &matrix[i-1][j];
		}
		else
		{
			if(i == 0)
			{
				if(matrix[i+1][j].occupied == 0) down = &matrix[i+1][j];
				if(matrix[i][j+1].occupied == 0) right = &matrix[i][j+1];
				if(matrix[i][j-1].occupied == 0) left = &matrix[i][j-1];
			}
			else if(i == row-1)
			{
				if(matrix[i-1][j].occupied == 0) up = &matrix[i-1][j];
				if(matrix[i][j+1].occupied == 0) right = &matrix[i][j+1];
				if(matrix[i][j-1].occupied == 0) left = &matrix[i][j-1];
			}
			else if(j == 0)
			{
				if(matrix[i+1][j].occupied == 0) down = &matrix[i+1][j];
				if(matrix[i-1][j].occupied == 0) up = &matrix[i-1][j];
				if(matrix[i][j+1].occupied == 0) right = &matrix[i][j+1];
			}
			else if(j == col-1)
			{
				if(matrix[i+1][j].occupied == 0) down = &matrix[i+1][j];
				if(matrix[i-1][j].occupied == 0) up = &matrix[i-1][j];
				if(matrix[i][j-1].occupied == 0) left = &matrix[i][j-1];
			}
			else
			{
				if(matrix[i+1][j].occupied == 0) down = &matrix[i+1][j];
				if(matrix[i-1][j].occupied == 0) up = &matrix[i-1][j];
				if(matrix[i][j-1].occupied == 0) left = &matrix[i][j-1];
				if(matrix[i][j+1].occupied == 0) right = &matrix[i][j+1];
			}
		}
	}

	std::string printCoors()
	{
		std::string output = std::string("( ") + patch::to_string(index_i) + ", " + patch::to_string(index_j) + ", ) ";
		if(up != NULL)
			output += "( " + patch::to_string(up->index_i) + ", " + patch::to_string(up->index_j) + ", ) ";
		else
			output += "( , , ) ";

		if(right != NULL)
			output += "( " + patch::to_string(right->index_i) + ", " + patch::to_string(right->index_j) + ", ) ";
		else
			output += "( , , ) ";

		if(down != NULL)
			output += "( " + patch::to_string(down->index_i) + ", " + patch::to_string(down->index_j) + ", ) ";
		else
			output += "( , , ) ";

		if(left != NULL)
			output += "( " + patch::to_string(left->index_i) + ", " + patch::to_string(left->index_j) + ", ) ";
		else
			output += "( , , ) ";
		return output;
	}
};

class Graph
{
	public:
		std::vector<Node*> vertexList; //stored all the vertexes in graph

		//2 nodes stored in vertex1 and vertex2 share same index have connected edge in with the cost stored in the same index location in edgeCost
		//example: vertex1[0] connects with vertex2[0] with the edge cost of edgeCost[0]
		std::vector<Node*> vertex1;
		std::vector<int> edgeCost;
		std::vector<Node*> vertex2;

	Graph(){}

	Graph(std::string txtFile);
	
	bool checkEdge(Node node1, Node node2) //LINEAR search
	{
		for(int i = 0; i < vertex1.size(); i++)
		{
			if(vertex1[i]->index_i == node1.index_i && vertex1[i]->index_j == node1.index_j) //found node1 in vertex1 vector
				if(vertex2[i]->index_i == node2.index_i && vertex2[i]->index_j == node2.index_j) //also found node2 in vertex2 vector at same location i
					return true; //edge exist, return true
			if(vertex1[i]->index_i == node2.index_i && vertex1[i]->index_j == node2.index_j) //found node2 in vertex1 vector
				if(vertex2[i]->index_i == node1.index_i && vertex2[i]->index_j == node1.index_j) //also found node1 in vertex2 vector at same location i
					return true;
		}
		return false;
	}

	int searchEdge(Node node1, Node node2) //LINEAR search
	{
		for(int i = 0; i < vertex1.size(); i++)
		{
			if(vertex1[i]->index_i == node1.index_i && vertex1[i]->index_j == node1.index_j) //found node1 in vertex1 vector
				if(vertex2[i]->index_i == node2.index_i && vertex2[i]->index_j == node2.index_j) //also found node2 in vertex2 vector at same location i
					return i; //edge exist, return i
			if(vertex1[i]->index_i == node2.index_i && vertex1[i]->index_j == node2.index_j) //found node2 in vertex1 vector
				if(vertex2[i]->index_i == node1.index_i && vertex2[i]->index_j == node1.index_j) //also found node1 in vertex2 vector at same location i
					return i;
		}
		return -1;
	}

	int searchVertex(Node node)
	{
		for(int i = 0; i < vertexList.size(); i++)
		{
			if(vertexList[i]->index_i == node.index_i && vertexList[i]->index_j == node.index_j)
				return i;
		}
		return -1;
	}

	int searchVertex(int index_i, int index_j)
	{
		for(int i = 0; i < vertexList.size(); i++)
		{
			if(vertexList[i]->index_i == index_i && vertexList[i]->index_j == index_j)
				return i;
		}
		return -1;
	}

	void addVertex(Node node)
	{
		if(searchVertex(node) == -1)
		{
			vertexList.push_back(new Node);
			vertexList.back()->index_i = node.index_i;
			vertexList.back()->index_j = node.index_j;
			vertexList.back()->occupied = node.occupied;
		}
	}

	int search_and_addVertex(Node node)
	{
		int i = searchVertex(node);
		if(i == -1)
		{
			vertexList.push_back(new Node);
			vertexList.back()->index_i = node.index_i;
			vertexList.back()->index_j = node.index_j;
			vertexList.back()->occupied = node.occupied;
			return vertexList.size()-1;
		}
		else
			return i;
	}

	void addEdge(Node node1, Node node2, int type, int cost) //type: type of connection from node1 to node2: 0 = up, 1 = right, 2 = down, 3 = left
	{
		if(!checkEdge(node1, node2))
		{
			int i1 = search_and_addVertex(node1);
			int i2 = search_and_addVertex(node2);
			
			vertex1.push_back(vertexList[i1]);
			vertex2.push_back(vertexList[i2]);
			edgeCost.push_back(cost);
			switch(type)
			{
				case 0:
				{
					vertex1.back()->up = vertex2.back();
					vertex2.back()->down = vertex1.back();
					break;
				}
				case 1:
				{
					vertex1.back()->right = vertex2.back();
					vertex2.back()->left = vertex1.back();
					break;
				}
				case 2:
				{
					vertex1.back()->down = vertex2.back();
					vertex2.back()->up = vertex1.back();
					break;
				}
				case 3:
				{
					vertex1.back()->left = vertex2.back();
					vertex2.back()->right = vertex1.back();
					break;
				}
			}
		}
	}
};

Graph::Graph(std::string txtFile)
{
	std::ifstream file;
	file.open(txtFile.c_str());
	if(file.is_open())
	{
		while(!file.eof())
		{
			std::string trash, coor1, coor2;
			Node central, up, right, down, left;
			bool bug = false;
			//central node
			getline(file, trash, ' ');
			getline(file, coor1, ' ');
			getline(file, coor2, ' ');
			getline(file, trash, ' ');
			//if(coor1 == "1," && coor2 == "1,") bug = true;
			if(coor1.substr(0, coor1.size()-1).c_str() != "")
			{
				central = Node(std::atoi(coor1.substr(0, coor1.size()-1).c_str()), std::atoi(coor2.substr(0, coor2.size()-1).c_str()), 0);
			}

			//up
			getline(file, trash, ' ');
			getline(file, coor1, ' ');
			getline(file, coor2, ' ');
			getline(file, trash, ' ');
			//if(bug) std::cout << coor1 << ", " << coor2 << "	";
			if(coor1.size() != 1) //coor1 and coor2 only has 1 char ',' if there are no value
			{
				up = Node(std::atoi(coor1.substr(0, coor1.size()-1).c_str()), std::atoi(coor2.substr(0, coor2.size()-1).c_str()), 0);
				addEdge(central, up, 0, 1);
			}

			//right
			getline(file, trash, ' ');
			getline(file, coor1, ' ');
			getline(file, coor2, ' ');
			getline(file, trash, ' ');
			//if(bug) std::cout << coor1 << ", " << coor2 << "	";
			if(coor1.size() != 1)
			{
				right = Node(std::atoi(coor1.substr(0, coor1.size()-1).c_str()), std::atoi(coor2.substr(0, coor2.size()-1).c_str()), 0);
				addEdge(central, right, 1, 1);
			}

			//down
			getline(file, trash, ' ');
			getline(file, coor1, ' ');
			getline(file, coor2, ' ');
			getline(file, trash, ' ');
			//if(bug) std::cout << coor1 << ", " << coor2 << "	";
			if(coor1.size() != 1)
			{
				down = Node(std::atoi(coor1.substr(0, coor1.size()-1).c_str()), std::atoi(coor2.substr(0, coor2.size()-1).c_str()), 0);
				addEdge(central, down, 2, 1);
			}

			//left
			getline(file, trash, ' ');
			getline(file, coor1, ' ');
			getline(file, coor2, ' ');
			getline(file, trash, ' ');
			//if(bug) std::cout << coor1 << ", " << coor2 << "	\n";
			if(coor1.size() != 1)
			{
				left = Node(std::atoi(coor1.substr(0, coor1.size()-1).c_str()), std::atoi(coor2.substr(0, coor2.size()-1).c_str()), 0);
				addEdge(central, left, 3, 1);
			}
		}
	}
	file.close();
};

int checkNode(std::vector<Node*> array, Node node)
{
	for(int i = 0; i < array.size(); i++)
		if(array[i]->index_i == node.index_i && array[i]->index_j == node.index_j)
			return i;
	return -1;
}

int checkNode(std::vector<Node*> array, int index_i, int index_j)
{
	for(int i = 0; i < array.size(); i++)
		if(array[i]->index_i == index_i && array[i]->index_j == index_j)
			return i;
	return -1;
}

int checkDirectionOfNeighbor(Node host, Node neighbor) //up = 0, right = 1, down = 2, left = 3
{
	if(host.up != NULL)
		if(host.up->index_i == neighbor.index_i && host.up->index_j == neighbor.index_j)
			return 0;
	if(host.right != NULL)
		if(host.right->index_i == neighbor.index_i && host.right->index_j == neighbor.index_j)
			return 1;
	if(host.down != NULL)
		if(host.down->index_i == neighbor.index_i && host.down->index_j == neighbor.index_j)
			return 2;
	if(host.left != NULL)
		if(host.left->index_i == neighbor.index_i && host.left->index_j == neighbor.index_j)
			return 3;
	return -1; //not a neighbor
}

#endif