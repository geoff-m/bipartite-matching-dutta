#include "Header.h"

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <Point3D.h>
#include "main.h"

BipartiteMatcher::BipartiteMatcher(int creatorId)
{
	matcherId = creatorId;
}

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

void BipartiteMatcher::addSelf(int myId, int x, int y, int cost)
{
	// Set the cost of this alternative.
    int id = toCellID(x, y);
	weight[myId][id] = cost;
	printf("Matcher %d: Added self (%d, %d) cost=%d\n", matcherId, x, y, cost);
	robotAtLocation[id] = -1;
}

void BipartiteMatcher::addAlternative1(int robotId, int x, int y, int cost)
{
    int id = toCellID(x, y);
    weight[robotId][id] = cost;
    printf("Matcher %d: Added alt for robot %d: (%d, %d) cost=%d\n", matcherId, robotId, x, y, cost);
	/*if (isSelf)
	{
		// Store the weight of this alternative.
		weight[SELF_INDEX][id] = cost;
        printf("Matcher %d: Added self alt (%d, %d) cost=%d\n", matcherId, x, y, cost);
	} else {
		// Store the weight of this alternative.
		weight[OTHER_INDEX][id] = cost;
        printf("Matcher %d: Added other alt (%d, %d) cost=%d\n", matcherId, x, y, cost);
	}*/
    robotAtLocation[id] = -1;
}

void BipartiteMatcher::addAlternative2(int robotId, int x, int y, int cost)
{
    addAlternative1(robotId, x, y, cost);
}

void BipartiteMatcher::display_matching() const {
	printf("\nRobot\tMatched Location\n");
	for (const auto& kvp : matching) {
		int r = kvp.first;
        Point p = toXY(kvp.second);
		printf("%d\t(%d, %d)\n", r, p.X, p.Y);
	}
}

void BipartiteMatcher::displayWeights() const
{
	for (int r = 0; r < ROBOT_COUNT; ++r)
	{
        printf("Matcher %d: Robot %d", matcherId, r);
        for (const auto& kvp : weight[r])
        {
            Point p = toXY(kvp.first);
            printf("\t(%d, %d)=%d", p.X, p.Y, kvp.second);
        }
        printf("\n");
	}
}


// Is this robot matched to any location?
bool BipartiteMatcher::isMatched_row(int robot) const
{
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
bool BipartiteMatcher::isMatched_col(int spot) const
{
    return robotAtLocation.count(spot) > 0 && robotAtLocation.at(spot) != -1;
}

int BipartiteMatcher::findBestSpot(int robot) const {
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
}

// For first pass.
// Which robot has the lowest cost to me ('spot')?
int BipartiteMatcher::findBestRobot(int spot) const {
	int row = -1;
	int min = INFINITE_COST;

	int weightToSpot[ROBOT_COUNT]; // each robot's weight to 'spot' (may be infinite for many)
    for (int r = 0; r < ROBOT_COUNT; r++) {
        if (weight[r].count(spot) > 0)
        {
            weightToSpot[r] = weight[r].at(spot);
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


int BipartiteMatcher::newFindBestSpot(int robot) const {
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
}

// For second pass.
int BipartiteMatcher::newFindBestRobot(int spot) const {
    int row = -1;
    int min = INFINITE_COST;

    int weightToSpot[ROBOT_COUNT]; // each robot's weight to 'spot' (may be infinite for many)
    for (int r = 0; r < ROBOT_COUNT; r++) {
        if (weight[r].count(spot) > 0 && !isMatched_row(r))
        {
            weightToSpot[r] = weight[r].at(spot);
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

//Manne's algorithm implementation
int BipartiteMatcher::solve_Manne() {
	int total_weight = 0;
	int cardinality = 0;

    printf("Begin solve %d\n", matcherId);

	//step 1: Initial 2-way matching.
	for (int row = 0; row < ROBOT_COUNT; row++) {// 'i' indices are used to indicate the left vertices
		int best_matched_col = findBestSpot(row);
		if (findBestRobot(best_matched_col) == row) {
		    Point p = toXY(best_matched_col);
			printf("Pass one: Mutual best match: robot %d -> cell (%d, %d)\n", row, p.X, p.Y);
			
			// Record matching.
			matching[row] = best_matched_col;
			robotAtLocation[best_matched_col] = row;
			
			total_weight += weight[row][best_matched_col];
			cardinality++;
		}
	}

	//printf("Finished initial pass of matching algo.\n");
	//display_matching();
	//printf("\n Cardinality here is: %d", cardinality);

	while (cardinality < ROBOT_COUNT) {
		for (int row = 0; row < ROBOT_COUNT; row++) {
			if (!isMatched_row(row)) { // this means that this row is not matched so far.
				int my_best_col = newFindBestSpot(row);
				if (newFindBestRobot(my_best_col) == row) {
                    Point p = toXY(my_best_col);
                    printf("\nPass two: Best match: robot %d -> cell (%d, %d)\n", row, p.X, p.Y);

					// Record matching.
					matching[row] = my_best_col;
					robotAtLocation[my_best_col] = row;
					
					total_weight += weight[row][my_best_col];
					cardinality++;
				}
			}
		}
	}

	display_matching();

    printf("End solve %d. Cardinality = %d. Cost = %d.\n", matcherId, cardinality, total_weight);

	return total_weight;
}

void BipartiteMatcher::solve()
{
	totalCost = solve_Manne();
}

int BipartiteMatcher::getTotalCost() const
{
	return totalCost;
}

bool BipartiteMatcher::hasResultFor(int robotId)
{
    return matching.count(robotId) > 0;
}

Point BipartiteMatcher::getResult(int robotId)
{
    int choice = -1;
    if (matching.count(robotId) == 0)
    {
        printf("Robot %d is not in the matching!\n", robotId);
    } else {
        choice = matching.at(robotId);
    }

	/*if (robot == 0)
	{
        if (matching.count(SELF_INDEX) == 0)
        {
            printf("Error: self-robot is not in the matching!\n");
        } else {
            choice = matching.at(SELF_INDEX);
        }
	}
	if (robot == 1)
	{
        if (matching.count(OTHER_INDEX) == 0)
        {
            printf("Error: other-robot is not in the matching!\n");
        } else {
            choice = matching.at(OTHER_INDEX);
        }
	}*/
    return toXY(choice);
}