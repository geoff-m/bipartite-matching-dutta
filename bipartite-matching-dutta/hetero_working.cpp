// Pothen_Fan_Matching.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <conio.h>
#include <random>
#include <ctime>
#include <time.h>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <algorithm> 
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <algorithm> 
#include <string>

using namespace std;

#define nodes 2 //number of agents (or nodes; #agents = #nodes)
#define MAX_STEPS 1000
//#define types 6
#define arena_length 10
#define arena_width 10

//int nodes;
//int nodes;
//int left[nodes];
//int right[nodes];
//int matching[nodes][nodes];

int weight[nodes][nodes];
int weight_copy[nodes][nodes];

int spot_position[nodes][2];
int agent_position[nodes][2];

int total_cost = 0;
std::vector<int> unmatched_nodes;

std::vector<int> comp_nodes_free;
std::vector<int> comp_nodes_unfree;
std::vector<int> neighbor_target_nodes;

std::vector<std::vector<int>> comp_graph;
std::vector<std::vector<int>> target_graph;

int type_array[nodes];
int matched_nodes[nodes];
int which_nodes_are_matched[nodes];
int cardinality = 0;
int total_weight = 0;
//int path_found=0;
//std::vector<std::vector<int>> compatibility;
//std::vector<int> types;
//std::vector<int> blocked_cols;
//std::vector<int> blocked_rows;
//static Random randomGenerator = new Random(System.currentTimeMillis());
std::vector<std::vector<int>> NEIGHBORS_T;// = new vector<vector<int>>();
std::vector<std::vector<int>> nodes_added;// = new vector<vector<int>>();
int blocked_cells[arena_length][arena_width];

void get_neighbors(int x, int y);

void finish_all() {
	for (int i = 0; i < arena_length; i++) {
		for (int j = 0; j < arena_width; j++) {
			blocked_cells[i][j] = 0;
		}
	}
	NEIGHBORS_T.clear();
	nodes_added.clear();
	comp_graph.clear();
	comp_nodes_free.clear();
	comp_nodes_unfree.clear();
	neighbor_target_nodes.clear();
	unmatched_nodes.clear();
	cardinality = 0;
	for (int i = 0; i < nodes; i++) {
		spot_position[i][0] = 0;
		agent_position[i][0] = 0;
		spot_position[i][1] = 0;
		agent_position[i][1] = 0;
		type_array[i] = 0;
		matched_nodes[i] = 0;
		which_nodes_are_matched[i] = 0;
		for (int j = 0; j < nodes; j++) {
			weight[i][j] = 0;
			weight_copy[i][j] = 0;
		}
	}

}

void generate_T() {
	//select root node;
	for (int i = 0; i < arena_length; i++) {
		//System.out.println("\n");
		for (int j = 0; j < arena_width; j++) {
			//System.out.println(Target.blocked_cells[i][j]);
			blocked_cells[i][j] = 0;
		}
		//System.out.println(" ");
	}
	int rooty;
	int rootx = rooty = arena_length / 2;
	int formed = 0;

	blocked_cells[rootx][rooty] = 1;

	std::vector<int> temp;// = new vector<int>();
	temp.push_back(rootx);
	temp.push_back(rooty);
	nodes_added.push_back(temp);
	spot_position[formed][0] = rootx;
	spot_position[formed][1] = rooty;
	formed++;
	//srand (time(NULL));
	while (formed < nodes) {
		NEIGHBORS_T.clear();
		int rand2 = rand() % nodes_added.size();//randomGenerator.nextInt(nodes_added.size());
		//System.out.println(nodes_added.size());
		get_neighbors(nodes_added.at(rand2).at(0), nodes_added.at(rand2).at(1));
		int rand1 = rand() % 4;//randomGenerator.nextInt(4);
		if (NEIGHBORS_T.at(rand1).size() > 0) {
			blocked_cells[NEIGHBORS_T.at(rand1).at(0)][NEIGHBORS_T.at(rand1).at(1)] = 1;
			std::vector<int> temp2;// = new vector<int>();
			temp2.push_back(NEIGHBORS_T.at(rand1).at(0));
			temp2.push_back(NEIGHBORS_T.at(rand1).at(1));
			nodes_added.push_back(temp2);
			spot_position[formed][0] = NEIGHBORS_T.at(rand1).at(0);
			spot_position[formed][1] = NEIGHBORS_T.at(rand1).at(1);
			formed++;
			//printf("\n Spot id %d and locationx %d and locationY %d",formed-1,NEIGHBORS_T.at(rand1).at(0),NEIGHBORS_T.at(rand1).at(1));
		}
	}

}

void setRobotPositions()
{
	agent_position[0][0] = 0;
	agent_position[0][1] = 0;

	agent_position[1][0] = 5;
	agent_position[1][1] = 0;
}

void generate_agent_pos() {
	int agent_count = 0;
	//srand (time(NULL));
	while (agent_count < nodes) {
		int rand_x = rand() % arena_length;
		int rand_y = rand() % arena_width;
		agent_position[agent_count][0] = rand_x;
		agent_position[agent_count][1] = rand_y;
		agent_count++;
	}
}

void get_neighbors(int x, int y) {
	// TODO Auto-generated method stub

	//vector<int> north=-1, south = -1, east=-1, west = -1;
	std::vector<int> north;// = new vector<int>();
	std::vector<int> south;// = new vector<int>();
	std::vector<int> east;// = new vector<int>();
	std::vector<int> west;// = new vector<int>();
	if (x == -1 || y == -1) {
		return;
	}
	if (x > 2 && blocked_cells[x - 1][y] == 0) {
		south.push_back(x - 1);
		south.push_back(y);
	}
	else {
		//south = -1;
	}
	if (y > 2 && blocked_cells[x][y - 1] == 0) {
		west.push_back(x);
		west.push_back(y - 1);
	}
	else {
		//west = -1;
	}
	if (x < arena_length - 2 && blocked_cells[x + 1][y] == 0) {
		north.push_back(x + 1);
		north.push_back(y);
	}
	else {
		//north = NULL;
	}
	if (y < arena_width - 2 && blocked_cells[x][y + 1] == 0) {
		east.push_back(x);
		east.push_back(y + 1);
	}
	else {
		//east = NULL;
	}

	NEIGHBORS_T.push_back(west);
	NEIGHBORS_T.push_back(east);
	NEIGHBORS_T.push_back(north);
	NEIGHBORS_T.push_back(south);
}

//generating random types for the nodes

void generate_types(int types) {
	//srand (time(NULL));
	//for(int this_robot=1;this_robot<nodes;this_robot++){
	//int rand_type = rand() % types + 1;  
	//type_array[this_robot]=rand_type;
	//}
	int robot_each_type = nodes / types;
	int this_type = 0;
	for (int this_robot = 0; this_robot < nodes; this_robot += (robot_each_type)) {
		//int rand_type = rand() % types + 1; 
		int count = 0;
		if (this_type + 1 <= types)
			this_type++;
		while (count <= robot_each_type - 1) {
			if (this_robot + count >= nodes)
				break;
			type_array[this_robot + count] = this_type;
			//printf("\n Robot %d 's type is %d ",this_robot+count,this_type);
			count++;
		}
		if (this_robot + count >= nodes)
			break;
	}
}

//reading graph from file
void read_Tgraph_from_file(std::string file_name) {
	// Open the file:
	std::ifstream fin(file_name);

	// Declare variables:
	int a, b, L;

	// Ignore headers and comments:
	while (fin.peek() == '%') fin.ignore(2048, '\n');

	// Read defining parameters:
	fin >> a >> b >> L;
	//nodes = a+1;
	//nodes = a+1;

	// Read the data
	for (int l = 0; l <= L; l++)
	{
		int m, n;
		//double data;
		fin >> m >> n;
		std::vector<int> one_edge;
		if (m != n) {
			one_edge.push_back(m);
			one_edge.push_back(n);
			//matrix[(m-1) + (n-1)*M] = data;
			target_graph.push_back(one_edge);
		}
		one_edge.clear();
	}
	//printf("\n Number of edges added in Target Graph: %d", target_graph.size());
	fin.close();

	//for(int i=0;i<target_graph.size();i++)
	//printf("\n new edge -> %d -- %d", target_graph.at(i).at(0),target_graph.at(i).at(1));

}

void create_line_Cgraph(int size) {
	std::vector<int> one_edge;
	for (int node1 = 1; node1 <= size; node1++) {
		one_edge.push_back(node1);
		one_edge.push_back(node1 + 1);
		comp_graph.push_back(one_edge);
		one_edge.clear();

		one_edge.push_back(node1);
		one_edge.push_back(node1);
		comp_graph.push_back(one_edge);
		one_edge.clear();
	}
}

void create_star_Cgraph(int size) {
	std::vector<int> one_edge;
	for (int node1 = 1; node1 <= size; node1++) {
		one_edge.push_back(size);
		one_edge.push_back(node1);
		comp_graph.push_back(one_edge);
		one_edge.clear();

		one_edge.push_back(node1);
		one_edge.push_back(node1);
		comp_graph.push_back(one_edge);
		one_edge.clear();
	}
}

void create_wheel_Cgraph(int size) {
	std::vector<int> one_edge;
	for (int node1 = 1; node1 <= size; node1++) {
		one_edge.push_back(size);
		one_edge.push_back(node1);
		comp_graph.push_back(one_edge);
		one_edge.clear();

		one_edge.push_back(node1);
		one_edge.push_back(node1);
		comp_graph.push_back(one_edge);
		one_edge.clear();

		if (node1 < size - 1) {
			one_edge.push_back(node1);
			one_edge.push_back(node1 + 1);
			comp_graph.push_back(one_edge);
			one_edge.clear();
		}
	}
	one_edge.push_back(size - 1);
	one_edge.push_back(1);
	comp_graph.push_back(one_edge);
	one_edge.clear();
}

void create_complete_Cgraph(int size) {

	for (int node1 = 1; node1 <= size; node1++) {
		for (int node2 = 1; node2 <= size; node2++) {
			//if(node1!=node2){
			std::vector<int> one_edge;
			one_edge.push_back(node1);
			one_edge.push_back(node2);
			comp_graph.push_back(one_edge);
			one_edge.clear();
			//}
		}
	}
}

void generate_Cgraph(int size, char character) {
	if (character == 'l') {
		create_line_Cgraph(size);
	}
	else if (character == 's') {
		create_star_Cgraph(size);
	}
	else if (character == 'c') {
		create_complete_Cgraph(size);
	}
	else if (character == 'w') {
		create_wheel_Cgraph(size);
	}
}

// from Hungarian algo Code!!
int** array_to_matrix(int* m, int rows, int cols) {
	int i, j;
	int** r;
	r = (int**)calloc(rows, sizeof(int*));
	for (i = 0; i < rows; i++)
	{
		r[i] = (int*)calloc(cols, sizeof(int));
		for (j = 0; j < cols; j++) {
			//int val = 
			r[i][j] = m[i*cols + j];
		}
	}

	return r;
}

void calculate_weights() {
	for (int i = 0; i < nodes; i++) {
		for (int j = 0; j < nodes; j++) {
			weight[i][j] = abs(agent_position[i][0] - spot_position[j][0]) + abs(agent_position[i][1] - spot_position[j][1]);
			weight_copy[i][j] = weight[i][j];
			//blocked_cells[i][j]=0;
		}
	}
}

//We assume that the bipartite graph is completely connected
void initialize() {
	srand(time(NULL));
	for (int i = 0; i < nodes; i++) {
		//left[i]=-1;
		//right[i]=-1;// -1 indicates no matching, >=0 indicates matched vertex.
		//visited[i]=0;
		matched_nodes[i] = -1;
		which_nodes_are_matched[i] = -1;
		//for(int j=0;j<nodes;j++){
		//matching[i][j]=0;// 0 indicates no matching and 1 indicated matched nodes, i.e., edge
		//weight[i][j]=rand()%10+1;// weights are randomized between 1 and 10
		//weight_copy[i][j]=weight[i][j];
		//}
	}

}

void display_matching() {
	printf("\n nodes --> nodes");
	for (int i = 0; i < nodes; i++) {
		//for(int j=1;j<nodes;j++){
		//if(weight[i][j]==-1){
		printf("\n %d --> %d", i, matched_nodes[i]);
		//}
		//}
	}
}

void display_weight() {
	//printf("\n left --> right");
	for (int i = 0; i < nodes; i++) {
		for (int j = 0; j < nodes; j++) {
			//if(matching[i][j]==1){
			printf("%d ", weight[i][j]);
			//}
		}
		printf("\n");
	}
}

int isMatched_row(int robot) {
	for (int i = 0; i < nodes; i++) {
		if (matched_nodes[i] == robot)
			return i;
	}
	return -1;
}

int isMatched_col(int spot) {
	//for(int i=0;i<nodes;i++){// 'i' indices are used to indicate the left vertices
	if (matched_nodes[spot] != -1)
		return matched_nodes[spot];
	//}
	else
		return -1;
}

int find_best_col(int row) {
	int col = -1;
	int min = 9999;
	for (int j = 0; j < nodes; j++) {
		if (weight[row][j] < min) {
			min = weight[row][j];
			col = j;
		}
	}
	return col;
}

int find_best_col_2(int row, int blocked_col) {
	int col = -1;
	int min = 9999;
	for (int j = 0; j < nodes; j++) {
		if (weight[row][j] < min && j != blocked_col) {
			min = weight[row][j];
			col = j;
		}
	}
	return col;
}

int find_best_row(int col) {
	int row = -1;
	int min = 9999;
	for (int i = 0; i < nodes; i++) {
		if (weight[i][col] < min) {
			min = weight[i][col];
			row = i;
		}
	}
	return row;
}

int find_best_row_2(int col, int blocked_row) {
	int best_row = -1;
	int min = 9999;
	for (int row = 0; row < nodes; row++) {
		if (weight[row][col] < min && row != blocked_row) {
			min = weight[row][col];
			best_row = row;
		}
	}
	//printf("\n returning best row: %d", best_row);
	return best_row;
}

int new_find_best_col(int row) {
	int col = -1;
	int min = 9999;
	for (int j = 0; j < nodes; j++) {
		if (weight[row][j] < min && isMatched_col(j) == -1) {
			min = weight[row][j];
			col = j;
		}
	}
	return col;
}

int new_find_best_row(int col) {
	int best_row = -1;
	int min = 9999;
	for (int row = 0; row < nodes; row++) {
		if (weight[row][col] < min && isMatched_row(row) == -1) {
			min = weight[row][col];
			best_row = row;
		}
	}
	//printf("\n returning best row: %d", best_row);
	return best_row;
}

//find the type of a certain robot
int find_robot_type(int robot) {
	int type = -1;
	if (robot == -1) {
		//printf("\n robot is -1!!!");
		return -1;
	}
	type = type_array[robot];
	return type;
}

//find which robot is matched to a certain spot
int find_who_matched(int spot) {
	int robot = -1;
	if (spot == -1)
		robot = -1;
	else
		robot = matched_nodes[spot];
	return robot;
}
void find_and_add_to_neighbors(int x, int y) {
	//printf("\n searching for x %d and y %d",x,y);
	for (int i = 0; i < nodes; i++) {
		//printf("\n This spot %d is -->  x %d and y %d",i,spot_position[i][0],spot_position[i][1]);
		if (spot_position[i][0] == x && spot_position[i][1] == y) {
			neighbor_target_nodes.push_back(i);
			//printf("\n printing spot id here.. %d",i);
		}

	}
}
//given a spot in the target graph, find the neighbor nodes
void get_neighbors_target(int spot) {//returns the neighbor nodes in target

	neighbor_target_nodes.clear();
	/*
	for(int i=0;i<target_graph.size();i++){
	if(target_graph.at(i).at(0)==spot){
	neighbor_target_nodes.push_back(target_graph.at(i).at(1));
	}
	if(target_graph.at(i).at(1)==spot){
	neighbor_target_nodes.push_back(target_graph.at(i).at(0));
	}

	//if(target_graph.at(i).at(1)>spot){
	//break;
	//}

	}
	*/
	//printf("\n coming here...");
	int x = spot_position[spot][0];
	int y = spot_position[spot][1];
	if (blocked_cells[x + 1][y] == 1) {
		//printf("\n coming here...1");
		find_and_add_to_neighbors(x + 1, y);
	}
	if (blocked_cells[x - 1][y] == 1) {
		//printf("\n coming here...2");
		find_and_add_to_neighbors(x - 1, y);
	}
	if (blocked_cells[x][y + 1] == 1) {
		//printf("\n coming here...3");
		find_and_add_to_neighbors(x, y + 1);
	}
	if (blocked_cells[x][y - 1] == 1) {
		//printf("\n coming here...4");
		find_and_add_to_neighbors(x, y - 1);
	}
}

//check compatibility between two types
bool check_comp(int my_type, int type2) {
	return true;
	/*
	if(type2==-1)
		return true;

	for(int i=0;i<comp_graph.size();i++){
		if((comp_graph.at(i).at(0)==my_type && comp_graph.at(i).at(1)==type2)
			||
			(comp_graph.at(i).at(1)==my_type && comp_graph.at(i).at(0)==type2)){
				return true;
		}

	}
	return false;
	*/
}

//find the degree of a type node in the compatibility graph
int find_degree_of_type(int type) {
	int degree = 0;
	for (int i = 0; i < comp_graph.size(); i++) {
		if ((comp_graph.at(i).at(0) == type || comp_graph.at(i).at(1) == type)) {
			degree++;
		}
	}
	return degree;
}

//check whether the spot is compatible to the robot's type or not.
bool is_compatible(int spot, int robot) {
	return true;
	/*
	std::vector<int> neighbor_types;
	int type=-1;
	int neighbor_robot =-1;
	int comp_flag = 0;
	int my_type = find_robot_type(robot);
	//neighbor_target_nodes.clear();
	//comp_nodes_free.clear();
	//comp_nodes_unfree.clear();

	get_neighbors_target(spot);
	for(int neighbor=0; neighbor<neighbor_target_nodes.size();neighbor++){
		neighbor_robot = find_who_matched(neighbor_target_nodes.at(neighbor));
		type = find_robot_type(neighbor_robot);
		neighbor_types.push_back(type);
	}
	for(int type_count=0; type_count<neighbor_types.size(); type_count++){
		if(check_comp(my_type, neighbor_types.at(type_count))==false){
			//printf("\n Compatibility is FALSE here, because types are: %d, %d", my_type, neighbor_types.at(type_count));
			comp_flag=1;
		}
		else{
			//printf("\n Compatibility is TRUE here, because types are: %d, %d", my_type, neighbor_types.at(type_count));
		}
	}

	//neighbor_target_nodes.clear();
	//neighbor_types.clear();

	if(comp_flag==0)
		return true;
	else
		return false;
		*/
}

//find comatible nodes for a robot
void find_compatible_nodes(int robot) {
	//printf("\n coming here? \n");
	std::vector<int> neighbor_types;
	int type = -1;
	int neighbor_robot = -1;
	int comp_flag = 0;
	int my_type = find_robot_type(robot);
	//neighbor_target_nodes.clear();
	//comp_nodes_free.clear();
	//comp_nodes_unfree.clear();

	//neighbor_target_nodes.clear();
	//neighbor_types.clear();

	for (int spot = 1; spot < nodes; spot++) {
		get_neighbors_target(spot);
		for (int neighbor = 0; neighbor < neighbor_target_nodes.size(); neighbor++) {
			neighbor_robot = find_who_matched(neighbor_target_nodes.at(neighbor));
			type = find_robot_type(neighbor_robot);
			neighbor_types.push_back(type);
		}
		//type=-1;
		//neighbor_robot =-1;
		for (int type_count = 0; type_count < neighbor_types.size(); type_count++) {
			if (check_comp(my_type, neighbor_types.at(type_count)) == false) {
				//printf("\n two types FALSE are: %d, %d \n", my_type, neighbor_types.at(type_count));
				comp_flag = 1;
			}
		}
		if (comp_flag == 0 && find_who_matched(spot) == -1) {
			comp_nodes_free.push_back(spot);
		}
		if (comp_flag == 0 && find_who_matched(spot) != -1) {
			comp_nodes_unfree.push_back(spot);
		}

		//neighbor_target_nodes.clear();
		neighbor_types.clear();
	}
	//printf("\n size of FREE COMPATIBLE nodes: %d \n", comp_nodes_free.size());
	//printf("\n size of UN-FREE COMPATIBLE nodes: %d \n", comp_nodes_unfree.size());
	//comp_nodes_free.clear();
	//comp_nodes_unfree.clear();

}

//find my best spot from the free compatible nodes
int find_best_from_free() {
	int best_spot = -1;
	int type = -1;
	int neighbor_robot = -1;
	std::vector<int> neighbor_types;
	int min_degree = 99999;
	int current_degree = 0;

	//neighbor_target_nodes.clear();
	//neighbor_types.clear();

	for (int spot = 0; spot < comp_nodes_free.size(); spot++) {
		get_neighbors_target(comp_nodes_free.at(spot));
		for (int neighbor = 0; neighbor < neighbor_target_nodes.size(); neighbor++) {
			neighbor_robot = find_who_matched(neighbor_target_nodes.at(neighbor));
			type = find_robot_type(neighbor_robot);
			neighbor_types.push_back(type);
		}
		for (int neighbor = 0; neighbor < neighbor_types.size(); neighbor++) {
			current_degree = current_degree + find_degree_of_type(neighbor_types.at(neighbor));
		}

		if (current_degree < min_degree && find_who_matched(spot) == -1) {
			min_degree = current_degree;
			best_spot = spot;
		}
		//neighbor_target_nodes.clear();
		neighbor_types.clear();
	}

	return best_spot;
}

//find my best spot from the set of non-free compatible nodes
int find_best_from_all() {
	int best_spot = -1;
	int type = -1;
	int matched_robot = -1;
	int max_degree = 0;
	int current_degree;
	//std::vector<int> neighbor_types;

	for (int spot = 0; spot < comp_nodes_unfree.size(); spot++) {
		matched_robot = find_who_matched(comp_nodes_unfree.at(spot));
		type = find_robot_type(matched_robot);
		current_degree = find_degree_of_type(type);
		//printf("\n matched robot is %d and degree here is: %d \n",matched_robot, current_degree);
		if (current_degree > max_degree) {
			max_degree = current_degree;
			best_spot = spot;
		}
	}
	return best_spot;
}

//Manne's algorithm implementation
void solve_Manne() {

	//step 1: Initial 2-way matching!
	clock_t tStart = clock();
	// vector Dset = null;
	for (int row = 0; row < nodes; row++) {// 'i' indices are used to indicate the left vertices
		int best_matched_col = find_best_col(row);
		if (find_best_row(best_matched_col) == row && is_compatible(best_matched_col, row) == true) {
			//printf("\n Matched row and col are: (%d,%d)", row, best_matched_col);
			//matching[row][best_matched_col]=1;
			total_cost = total_cost + weight[row][best_matched_col];
			//weight[row][best_matched_col] = -1;
			matched_nodes[best_matched_col] = row;
			which_nodes_are_matched[row] = 1;
			total_weight = total_weight + weight[row][best_matched_col];
			cardinality++;
			//printf("\n spot % is matched to agent %d",best_matched_col,row);
			//Dset = Dset U {row, (x,y)}
		}
	}
	// && is_compatible(best_matched_col, row)==true 
	// && is_compatible(my_best_col, row)==true

	//printf("\n Cardinality here is: %d", cardinality);

	int last_card = cardinality;
	//while(size(Dset)>0){
	while (cardinality <= nodes) {
		for (int row = 0; row < nodes; row++) {
			if (isMatched_row(row) == -1) {// this means that this row is not matched so far!
				int my_best_col = new_find_best_col(row);
				if (new_find_best_row(my_best_col) == row && is_compatible(my_best_col, row) == true) {
					total_cost = total_cost + weight[row][my_best_col];
					matched_nodes[my_best_col] = row;
					total_weight = total_weight + weight[row][my_best_col];
					which_nodes_are_matched[row] = 1;
					//printf("\n spot % is matched to agent %d",my_best_col,row);
					cardinality++;
					if (cardinality > nodes)
						break;
				}
				else if (is_compatible(my_best_col, row) == false) {
					//printf("\n WHY AM I GETTING A FALSE HERE??");
				}
			}
		}
		if (cardinality == last_card)
			break;
		else
			last_card = cardinality;

		if (cardinality > nodes)
			break;
	}
	printf("\n At the end of Manne's algorithm, values of cardinality and nodes are: %d, %d", cardinality, nodes);
	printf("\n End Manne's algorithm. All matching done (F.MANNE) -- total cost is: %d with cardinality %d", total_cost,cardinality);
	display_matching();

	//printf("\n Time taken for Manne's Algo: %.2f ms.\n", (double)(tEnd_Manne - tStart)*1000/CLOCKS_PER_SEC);
}

//******new function to store compatible nodes
void find_nodes_compatible(int robot) {
	for (int spot = 0; spot < nodes; spot++) {
		if (is_compatible(spot, robot) == true) {
			if (matched_nodes[spot] == -1) {
				comp_nodes_free.push_back(spot);
			}
			else {
				comp_nodes_unfree.push_back(spot);
			}
		}
	}
}

int find_spot_id(int x, int y) {
	for (int i = 0; i < nodes; i++) {
		if (spot_position[i][0] == x && spot_position[i][1] == y) {
			return i;
		}
	}
	return -1;
}

void print_allocation_types() {
	for (int i = 0; i < arena_length; i++) {
		for (int j = 0; j < arena_length; j++) {
			if (blocked_cells[i][j] == 0) {
				printf(" 0 ");
			}
			else {
				int spot = find_spot_id(i, j);
				if (spot == -1)
					printf("\n why spot is -1??? (%d,%d)", i, j);
				int robot = find_who_matched(spot);
				int type = find_robot_type(robot);
				printf(" %d ", type);
				if (type == -1) {
					//printf("\n why?? spot %d and robot %d",spot,robot);
				}
			}
		}
		printf("\n");
	}

}
//**********main function starts here
int _tmain(int argc, char* argv[])
{
	//experimental settings
	//std::string file_name = "target1_39.mtx";
	//read_Tgraph_from_file(file_name);

	//char cgraph_list[] = {'c','l','s','w'};
	//int type_list[]={2,3,4,5,6};
	//int runs = 10;

			//ofstream outFile;
			////std::string str1 = to_string(static_cast<long long>(cgraph_list[cl]));
			//stringstream ss;
			//std::string str1;
			//char c = cgraph_list[cl];
			//ss << c;
			//ss >> str1;
			//std::string fileName = "hetero_" + str1;//+ "_" + type_list[tl];
			//std::string str2 = to_string(static_cast<long long>(type_list[tl]));
			//std::string f2 = "_T" + str2;// + ".txt";
			//std::string f3 = ".txt";
			//fileName = fileName + f2 + f3;
			//int types = type_list[tl];
			//char cgraph = cgraph_list[cl];
			//
//			for(int rc=1;rc<=runs;rc++){
//				outFile.open(fileName, std::ios_base::app);

				//generate_types(types);//generate random types of nodes
				//generate_Cgraph(types, cgraph);// generate comaptibility graph

				//step 0: Initialization
	initialize();
	//generate_T(); // not needed?

	setRobotPositions(); // my replacement for generate_agent_pos
	//generate_agent_pos();

	calculate_weights();

	/* Manne's Algorithm Starts here */
	//clock_t tStart = clock();
	solve_Manne(); // Initial matching is done!

	/* Local search algorithm for Constraint Satisfaction starts here */
	for (int robot = 0; robot < nodes; robot++) {
		if (which_nodes_are_matched[robot] == -1) {
			//printf("\n Robot here is: %d\n", robot);
			unmatched_nodes.push_back(robot);
		}
	}
	//printf("\n Size of unmatched nodes vector is: %d\n", unmatched_nodes.size());

	int count = 0;
	//while(unmatched_nodes.size()>0 || count <= MAX_STEPS){
	while (true) {
		if (unmatched_nodes.size() == 0)
			break;
		if (count >= MAX_STEPS)
			break;
		count++;
		//if(count==MAX_STEPS)
		//printf("\n count is: %d\n", count);
		int robot = unmatched_nodes.at(unmatched_nodes.size() - 1);
		//printf("\n Current robot is: %d\n", robot);
		unmatched_nodes.pop_back();
		if (robot == -1) {
			//printf("\n WHY -1 is the robot here???? \n");
		}
		//find_compatible_nodes(robot);
		find_nodes_compatible(robot);//new function which finds and store the compatible nodes for the robot.
		int best_spot = -1;
		if (comp_nodes_free.size() != 0) {//finding nodes from free
			best_spot = find_best_from_free();
			//printf("\n best spot 1 here is: %d\n", best_spot);
			if (best_spot != -1 && robot != -1) {
				matched_nodes[best_spot] = robot;
				which_nodes_are_matched[robot] = 1;
				total_weight = total_weight + weight[robot][best_spot];
				cardinality++;
				//printf("\n from UNFREE -- spot % is matched to agent %d",best_spot,robot);
			}

		}
		else {// no compatible spot is free
			best_spot = find_best_from_all();
			if (best_spot == -1) {
				//printf("\n WHY -1 is the Best-Spot here???? \n");
			}
			//printf("\n best spot 2 here is: %d\n", best_spot);
			int this_matched_robot = find_who_matched(best_spot);
			unmatched_nodes.push_back(this_matched_robot);
			total_weight = total_weight - weight[this_matched_robot][best_spot];
			matched_nodes[best_spot] = robot;
			which_nodes_are_matched[robot] = 1;
			total_weight = total_weight + weight[robot][best_spot];
			//printf("\n From FREE -- spot % is matched to agent %d",best_spot,robot);
			//unmatched_nodes.erase(unmatched_nodes.at(0));

		}
		comp_nodes_free.clear();
		comp_nodes_unfree.clear();
	}
	//printf("\n cardinality is when algorithm STOPS: %d\n", cardinality);
	//clock_t tEnd_Manne = clock();
	//int total = 0;
	//for(int all=0;all<nodes;all++){
		//if(matched_nodes[all]!=-1){

		//}
	//}
	//printf("\n Time taken for Constrained satisfaction Algo: %.2f ms.\n", (double)(tEnd_Manne - tStart)*1000/CLOCKS_PER_SEC);
	//double time_taken = (double)(tEnd_Manne - tStart)*1000/CLOCKS_PER_SEC;
	//outFile << nodes << "," << rc << "," << types << "," << cgraph << "," << time_taken << "," << cardinality << "," << total_weight << endl;
	//outFile.close();
	total_weight = 0;
	finish_all();


	//getch();
	return 0;
}

