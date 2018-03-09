#pragma once

#include <stdio.h>

// main in main.cpp
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
	int myalts[3][2];
	int otheralts[2][2];

public:
	BipartiteMatcher();

	void solve();

	// Add own robot's original location.
	void addSelf(int x, int y, int cost);

	// Add either robot's first alternative.
	void addAlternative1(bool isSelf, int x, int y, int cost);

	// Add either robot's second alternative.
	void addAlternative2(bool isSelf, int x, int y, int cost);

	void displayWeights();

	// 0 for self, 1 for non-self.
	Point getResult(int robot);
};