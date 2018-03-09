#include "Header.h"

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

#define ROBOT_COUNT 2 //number of agents (or nodes; #agents = #nodes)
#define LOCATION_COUNT ROBOT_COUNT * 2 + 1

#define MAX_STEPS 1000
#define arena_length 10
#define arena_width 10

int weight[ROBOT_COUNT][3];

int agent_position[ROBOT_COUNT][2];
//int spot_position[ROBOT_COUNT][2];

std::vector<std::vector<int>> comp_graph;
std::vector<std::vector<int>> target_graph;

int matched_nodes[ROBOT_COUNT];

#define SELF_INDEX 0
#define OTHER_INDEX 1

#define FIRST_ALT_INDEX 0
#define SECOND_ALT_INDEX 1
#define ORIGINAL_LOCATION 2

#define INFINITE_COST 99999

void BipartiteMatcher::addSelf(int x, int y, int cost)
{
	// Set my position.
	agent_position[SELF_INDEX][0] = x;
	agent_position[SELF_INDEX][1] = y;

	// Store my initial position as one of my alternatives.
	myalts[ORIGINAL_LOCATION][0] = x;
	myalts[ORIGINAL_LOCATION][1] = y;
	
	// Set the cost of this alternative.
	weight[SELF_INDEX][ORIGINAL_LOCATION] = cost;
}

void BipartiteMatcher::addAlternative1(bool isSelf, int x, int y, int cost)
{
	if (isSelf)
	{
		// Store the alternative location.
		myalts[FIRST_ALT_INDEX][0] = x;
		myalts[FIRST_ALT_INDEX][1] = y;
		
		// Store the weight of this alternative.
		weight[SELF_INDEX][FIRST_ALT_INDEX] = cost;
	}
	else {
		// Store the alternative location.
		otheralts[FIRST_ALT_INDEX][0] = x;
		otheralts[FIRST_ALT_INDEX][1] = y;

		// Store the weight of this alternative.
		weight[OTHER_INDEX][FIRST_ALT_INDEX] = cost;
	}
}

void BipartiteMatcher::addAlternative2(bool isSelf, int x, int y, int cost)
{
	if (isSelf)
	{
		// Store the alternative location.
		myalts[SECOND_ALT_INDEX][0] = x;
		myalts[SECOND_ALT_INDEX][1] = y;

		// Store the weight of this alternative.
		weight[SELF_INDEX][SECOND_ALT_INDEX] = cost;
	}
	else {
		// Store the alternative location.
		otheralts[SECOND_ALT_INDEX][0] = x;
		otheralts[SECOND_ALT_INDEX][1] = y;

		// Store the weight of this alternative.
		weight[OTHER_INDEX][SECOND_ALT_INDEX] = cost;
	}
}

//reading graph from file
/**
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
*/


int robotsMatched[LOCATION_COUNT]; // maps location ("column") to robot.

void initialize() {
	for (int i = 0; i < ROBOT_COUNT; i++) {
		matched_nodes[i] = -1;
	}
	for (int i = 0; i < LOCATION_COUNT; ++i)
	{
		robotsMatched[i] = -1;
	}
}

void display_matching() {
	printf("\nrobot\tlocation\n");
	for (int i = 0; i < ROBOT_COUNT; i++) {

		printf("%d\t%d\n", i, matched_nodes[i]);
		//}
		//}
	}
}

void BipartiteMatcher::displayWeights()
{
	printf("robot\talt1\talt2\tstart\n");
	for (int i = 0; i < ROBOT_COUNT; ++i)
	{
		printf("%d\t%d\t%d\t%d\n", i,
			weight[i][0],
			weight[i][1],
			weight[i][2]);
	}
}


// Is this robot matched to any location?
bool isMatched_row(int robot) {
	for (int i = 0; i < ROBOT_COUNT; i++) {
		if (matched_nodes[i] == robot)
			return true;
	}
	return false;
}

/**int isMatched_col(int spot) {
	//for(int i=0;i<nodes;i++){// 'i' indices are used to indicate the left vertices
	//printf("\nIs location %d matched: %d.\n", v);
	if (matched_nodes[spot] != -1)
		return matched_nodes[spot];
	//}
	else
		return -1;
}
*/

// Is this location matched to any robot?
bool isMatched_col(int spot)
{
	return robotsMatched[spot] != -1;
	//return matched_nodes[spot] != -1;
}

int find_best_col(int row) {
	int col = -1;
	int min = 9999;
	for (int j = 0; j < 3; j++) {
		if (weight[row][j] < min) {
			min = weight[row][j];
			col = j;
		}
	}
	return col;
}

// For first pass.
int find_best_row(int col) {
	int row = -1;
	int min = 9999;
	for (int i = 0; i < ROBOT_COUNT; i++) {
		if (weight[i][col] < min) {
			min = weight[i][col];
			row = i;
		}
	}
	return row;
}


int new_find_best_col(int row) {
	//printf("\nEntered new_find_best_col(%d).\n", row);
	int col = -1;
	int min = 9999;
	
	for (int j = 0; j < 3; j++) {
		printf("isMatched_col(%d) returned %d.\n", j, isMatched_col(j));
		if (weight[row][j] < min && !isMatched_col(j)) {
			min = weight[row][j];
			col = j;
		}
	}
	return col;
}

// For second pass.
int new_find_best_row(int col) {
	int best_row = -1;
	int min = 9999;
	for (int row = 0; row < ROBOT_COUNT; row++) {
		if (weight[row][col] < min && !isMatched_row(row)) {
			min = weight[row][col];
			best_row = row;
		}
	}
	//printf("\n returning best row: %d", best_row);
	return best_row;
}


BipartiteMatcher::BipartiteMatcher()
{
	initialize();

	// Initialize all weights to infinity.
	for (int i = 0; i < ROBOT_COUNT; ++i)
	{
		weight[i][0] = INFINITE_COST;
		weight[i][1] = INFINITE_COST;
		weight[i][2] = INFINITE_COST;
	}
}

//Manne's algorithm implementation
void solve_Manne() {
	int total_cost = 0;
	int total_weight = 0;
	int cardinality = 0;

	//step 1: Initial 2-way matching!
	// vector Dset = null;
	for (int row = 0; row < ROBOT_COUNT; row++) {// 'i' indices are used to indicate the left vertices
		int best_matched_col = find_best_col(row);
		if (find_best_row(best_matched_col) == row) {
			printf("\n Matched row and col are: (%d,%d)\n", row, best_matched_col);
			total_cost = total_cost + weight[row][best_matched_col];
			
			// Record matching.
			matched_nodes[row] = best_matched_col;
			robotsMatched[best_matched_col] = row;
			
			total_weight = total_weight + weight[row][best_matched_col];
			cardinality++;
		}
	}

	printf("Finished initial pass of matching algo.\n");
	display_matching();

	//printf("\n Cardinality here is: %d", cardinality);

	int last_card = cardinality;
	while (cardinality <= ROBOT_COUNT) {
		for (int row = 0; row < ROBOT_COUNT; row++) {
			if (!isMatched_row(row)) {// this means that this row is not matched so far!
				int my_best_col = new_find_best_col(row);
				if (new_find_best_row(my_best_col) == row) {
					printf("\n Matched row and col are: (%d,%d)\n", row, my_best_col);
					total_cost = total_cost + weight[row][my_best_col];
					
					// Record matching.
					matched_nodes[row] = my_best_col;
					robotsMatched[my_best_col] = row;
					
					total_weight = total_weight + weight[row][my_best_col];
					cardinality++;
					if (cardinality > ROBOT_COUNT)
						break;
				}
			}
		}
		if (cardinality == last_card)
			break;
		else
			last_card = cardinality;

		if (cardinality > ROBOT_COUNT)
			break;
	}
	printf("\n At the end of Manne's algorithm, values of cardinality and nodes are: %d, %d\n", cardinality, ROBOT_COUNT);
	printf("\n End Manne's algorithm. All matching done (F.MANNE) -- total cost is: %d with cardinality %d\n", total_cost,cardinality);
	display_matching();

	//printf("\n Time taken for Manne's Algo: %.2f ms.\n", (double)(tEnd_Manne - tStart)*1000/CLOCKS_PER_SEC);
}

void BipartiteMatcher::solve()
{
	solve_Manne();
}

Point BipartiteMatcher::getResult(int robot)
{
	if (robot == 0)
	{
		int choice = matched_nodes[SELF_INDEX];
		return Point(myalts[choice][0], myalts[choice][1]);
	}
	if (robot == 1)
	{
		int choice = matched_nodes[OTHER_INDEX];
		_ASSERT(choice < 2);

		return Point(otheralts[choice][0], otheralts[choice][1]);
	}
}