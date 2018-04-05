#include "Header.h"

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <vector>
#include <map>
#include <Point3D.h>
#include "main.h"

#define ROBOT_COUNT 2 //number of agents (or nodes; #agents = #nodes)
#define LOCATION_COUNT ROBOT_COUNT * 2 + 1

//int weight[ROBOT_COUNT][3];
// we use cell IDs as columns in bipartite matching's weight array
std::map<int, int> weight[ROBOT_COUNT]; // each 'weight' map maps cell ID to cost.

std::map<int, int> robotAtLocation; // maps location ("column") to robot.
std::map<int, int> matching; // the result. maps robot to location.

int agent_position[ROBOT_COUNT][2];
//int spot_position[ROBOT_COUNT][2];

std::vector<std::vector<int>> comp_graph;
std::vector<std::vector<int>> target_graph;

#define SELF_INDEX 0
#define OTHER_INDEX 1

#define INFINITE_COST 99999


static int toCellID(int x, int y)
{
	return x * ROW_COLUMN_COUNT + y;
}

static Point toXY(int cellID)
{
	int x = (int)floor(cellID / ROW_COLUMN_COUNT);
	int y = cellID % ROW_COLUMN_COUNT;
	return Point(x, y);
}

void BipartiteMatcher::addSelf(int x, int y, int cost)
{
	// Set my position.
	agent_position[SELF_INDEX][0] = x;
	agent_position[SELF_INDEX][1] = y;

	// Set the cost of this alternative.
    int id = toCellID(x, y);
	weight[SELF_INDEX][id] = cost;
	robotAtLocation[id] = -1;
}

void BipartiteMatcher::addAlternative1(bool isSelf, int x, int y, int cost)
{
    int id = toCellID(x, y);
	if (isSelf)
	{
		// Store the weight of this alternative.
		weight[SELF_INDEX][id] = cost;
	}
	else {
		// Store the weight of this alternative.
		weight[OTHER_INDEX][id] = cost;
	}
    robotAtLocation[id] = -1;
}

void BipartiteMatcher::addAlternative2(bool isSelf, int x, int y, int cost)
{
    addAlternative1(isSelf, x, y, cost);
}

void initialize() {
	/*for (int i = 0; i < ROBOT_COUNT; i++) {
		matching[i] = -1;
	}*/

	/*for (int i = 0; i < LOCATION_COUNT; ++i)
	{
		robotAtLocation[i] = -1;
	}*/
}

void display_matching() {
	printf("\nRobot\tMatched Location\n");
	for (int r = 0; r < ROBOT_COUNT; r++) {
        Point p = toXY(matching[r]);
		printf("%d\t(%d, %d)\n", r, p.X, p.Y);
	}
}

void BipartiteMatcher::displayWeights()
{
	for (int r = 0; r < ROBOT_COUNT; ++r)
	{
        printf("Robot %d", r);
        for (const auto& kvp : weight[r])
        {
            Point p = toXY(kvp.first);
            printf("\t(%d, %d)=%d", p.X, p.Y, kvp.second);
        }
        printf("\n");
	}
}


// Is this robot matched to any location?
bool isMatched_row(int robot) {
    return matching.count(robot) > 0;
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
    return robotAtLocation.count(spot) > 0;
}

int findBestSpot(int robot) {
	int col = -1;
	int min = INFINITE_COST;
	for (const auto& kvp : weight[robot])
    {
        int thisWeight = kvp.second;
        if (thisWeight < min)
        {
            min = thisWeight;
            col = kvp.first;
        }
    }
    return col;
	for (int j = 0; j < 3; j++) {
		if (weight[robot][j] < min) {
			min = weight[robot][j];
			col = j;
		}
	}
	return col;
}

// For first pass.
// Which robot has the lowest cost to me ('spot')?
int findBestRobot(int spot) {
	int row = -1;
	int min = INFINITE_COST;

	int weightToSpot[ROBOT_COUNT]; // each robot's weight to 'spot' (may be infinite for many)
    for (int r = 0; r < ROBOT_COUNT; r++) {
        if (weight[r].count(spot) > 0)
        {
            weightToSpot[r] = weight[r][spot];
        } else {
            weightToSpot[r] = INFINITE_COST;
        }
    }

    // select the robot that has minimum weight.
    for (int r = 0; r < ROBOT_COUNT; r++) {
        int myWeight = weightToSpot[r];
        if (myWeight < min)
        {
            min = myWeight;
            row = r;
        }
    }
    return row;
}


int newFindBestSpot(int robot) {
	//printf("\nEntered newFindBestSpot(%d).\n", robot);
	int spot = -1;
	int min = INFINITE_COST;

    for (const auto& kvp : weight[robot])
    {
        int thisCost = kvp.second;
        if (!isMatched_col(kvp.first) && thisCost < min)
        {
            min = thisCost;
            spot = kvp.first;
        }
    }
    return spot;

	for (int j = 0; j < 3; j++) {
		printf("isMatched_col(%d) returned %d.\n", j, isMatched_col(j));
		if (weight[robot][j] < min && !isMatched_col(j)) {
			min = weight[robot][j];
			spot = j;
		}
	}
	return spot;

}

// For second pass.
int newFindBestRobot(int spot) {
    int row = -1;
    int min = INFINITE_COST;

    int weightToSpot[ROBOT_COUNT]; // each robot's weight to 'spot' (may be infinite for many)
    for (int r = 0; r < ROBOT_COUNT; r++) {
        if (weight[r].count(spot) > 0 && !isMatched_row(r))
        {
            weightToSpot[r] = weight[r][spot];
        } else {
            weightToSpot[r] = INFINITE_COST;
        }
    }

    // select the robot that has minimum weight.
    for (int r = 0; r < ROBOT_COUNT; r++) {
        int myWeight = weightToSpot[r];
        if (myWeight < min)
        {
            min = myWeight;
            row = r;
        }
    }
    return row;
}


BipartiteMatcher::BipartiteMatcher()
{
	initialize();

	/*// Initialize all weights to infinity.
	for (int i = 0; i < ROBOT_COUNT; ++i)
	{
		weight[i][0] = INFINITE_COST;
		weight[i][1] = INFINITE_COST;
		weight[i][2] = INFINITE_COST;
	}*/
}

//Manne's algorithm implementation
int solve_Manne() {
	int total_cost = 0;
	int total_weight = 0;
	int cardinality = 0;


	//step 1: Initial 2-way matching!
	// vector Dset = null;
	for (int row = 0; row < ROBOT_COUNT; row++) {// 'i' indices are used to indicate the left vertices
		int best_matched_col = findBestSpot(row);
		if (findBestRobot(best_matched_col) == row) {
		    Point p = toXY(best_matched_col);
			printf("\nPass one: Mutual best match: robot %d -> cell (%d, %d)\n", row, p.X, p.Y);

			total_cost = total_cost + weight[row][best_matched_col];
			
			// Record matching.
			matching[row] = best_matched_col;
			robotAtLocation[best_matched_col] = row;
			
			total_weight = total_weight + weight[row][best_matched_col];
			cardinality++;
		}
	}

	printf("Finished initial pass of matching algo.\n");
	display_matching();
	printf("\n Cardinality here is: %d", cardinality);

	int last_card = cardinality;
	while (cardinality <= ROBOT_COUNT) {
		for (int row = 0; row < ROBOT_COUNT; row++) {
			if (!isMatched_row(row)) {// this means that this row is not matched so far!
				int my_best_col = newFindBestSpot(row);
				if (newFindBestRobot(my_best_col) == row) {
                    Point p = toXY(my_best_col);
                    printf("\nPass two: Best match: robot %d -> cell (%d, %d)\n", row, p.X, p.Y);

                    total_cost = total_cost + weight[row][my_best_col];
					
					// Record matching.
					matching[row] = my_best_col;
					robotAtLocation[my_best_col] = row;
					
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
	//printf("\n End Manne's algorithm. All matching done (F.MANNE) -- total cost is: %d with cardinality %d\n", total_cost,cardinality);
	display_matching();


	//printf("\n Time taken for Manne's Algo: %.2f ms.\n", (double)(tEnd_Manne - tStart)*1000/CLOCKS_PER_SEC);
	return total_cost;
}

void BipartiteMatcher::solve()
{
    displayWeights();

	totalCost = solve_Manne();
}

int BipartiteMatcher::getTotalCost() const
{
	return totalCost;
}

Point BipartiteMatcher::getResult(int robot)
{
    int choice = -1;
	if (robot == 0)
	{
		choice = matching[SELF_INDEX];
	}
	if (robot == 1)
	{
		choice = matching[OTHER_INDEX];
	}
    return toXY(choice);
}