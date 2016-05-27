CSE180 final project by Lam Bui and Lucio Isaac Perez

Start everything: gazebo, load mode...
Run mapInfo node from finalproject pkg, and wait for it to be done

It will create 3 files in the directive the terminal is currently in: rawData.txt, nodeMap.txt, graph.txt

	rawData.txt is a file that contains unmordified matrix the map server provides
	nodeMap.txt is a file that turns patches of 10x10 data point in rawData into map nodes
		(0,0) (origin) is in the upper left corner

		nodeMap's coordinate frame:
		(0,0)--------------------------> x
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

		gazebo's coordinate frame for reference:
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

		so by looking at nodeMap data, the map looks flipped left to right.

	graph.txt is a file that contains all the relations of every unoccupied node to its 4 neighbors in this format:
		each line contains 4 nodes in this order:
			current-node, top-neighbor, right-neighbor, bottom-neighbor, left-neighbor
			if a neighbor is ( , , ) meaning that neighbor is occupied

From the same directory, open terminal and run findPath node from finalproject pkg.
	This node will use graph.txt and run dijktra algorithm to find the path from user-inputted start location and end location.
		index_i is x, and index_j is y
		Start location will be 20, 20
		Goal location: your choice, our goal location for testing is (1, 5)
		If it segfault then that means the end location is occupied, should try to run it again and input different goal location
	This node is not necessary to run, it just visualizes the path that the robot will run in the gazebo simulation

To run the robot in simulation, from the same directory containing graph.txt, run moveBot node in finalproject pkg right after running the launch file. The robot will try to get to the destination.
	Start node: 20,20
	End node: 1,5

	while the robot run the system will out put a line of 3 int and a line of 3 double
		the first 2 int is the node robot thinks it is in according to amcl_pose so if they change drastically then the robot is lost.
		and if it doesnt output DEVIATE that mean it is on track

