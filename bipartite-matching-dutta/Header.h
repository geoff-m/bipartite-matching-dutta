#pragma once

#include <stdio.h>
#include <map>
#include <vector>
#include <algorithm>
#include "main.h" // for ROBOT_COUNT

//#define ROBOT_COUNT 2 //number of agents (or nodes; #agents = #nodes)
// we use cell IDs as columns in bipartite matching's weight array

#define INFINITE_COST 99999

int main(int argc, char* argv[]);

struct Point
{
	int X;
	int Y;
public:
	Point(int x, int y)
	{
		X = x;
		Y = y;
	}
};


class BipartiteMatcher
{
private:
	std::map<int, int> weight[ROBOT_COUNT]; // each 'weight' maps cell ID to cost.

	std::map<int, int> robotAtLocation; // maps location ("column") to robot.
	std::map<int, int> matching; // the result. maps robot to location.

	int totalCost;
	int matcherId; // for debug. keeps track of which robot owns this instance.

	int findBestRobot(int spot) const;
	int findBestSpot(int robot) const;
	int newFindBestSpot(int robot) const;
	int newFindBestRobot(int spot) const;
	int solve_Manne();
	bool isMatched_row(int robot) const;
	bool isMatched_col(int spot) const;

	void display_matching() const;
public:
	BipartiteMatcher(int creatorId); // id is for debug only.

	void solve();

	// Gets the total cost (weight) of the computed matching.
	int getTotalCost() const;

	// Add own robot's original location.
    void addSelf(int myId, int x, int y, int cost);

	// Add a robot's first alternative.
	void addAlternative1(int robotId, int x, int y, int cost);

	// Add a robot's second alternative.
	void addAlternative2(int robotId, int x, int y, int cost);

	void displayWeights() const;

	bool hasResultFor(int robotId);

	Point getResult(int robotId);
};