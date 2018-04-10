#pragma once

#include <stdio.h>
#include <map>
#include <vector>
#include <algorithm>

#define ROBOT_COUNT 2 //number of agents (or nodes; #agents = #nodes)
// we use cell IDs as columns in bipartite matching's weight array

#define SELF_INDEX 0
#define OTHER_INDEX 1

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
	std::map<int, int> weight[ROBOT_COUNT]; // each 'weight' map maps cell ID to cost.

	std::map<int, int> robotAtLocation; // maps location ("column") to robot.
	std::map<int, int> matching; // the result. maps robot to location.

	int agent_position[ROBOT_COUNT][2];
	std::vector<std::vector<int>> comp_graph;
	std::vector<std::vector<int>> target_graph;

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
	void addSelf(int x, int y, int cost);

	// Add either robot's first alternative.
	void addAlternative1(bool isSelf, int x, int y, int cost);

	// Add either robot's second alternative.
	void addAlternative2(bool isSelf, int x, int y, int cost);

	void displayWeights() const;

	// 0 for self, 1 for non-self.
	Point getResult(int robot);
};