// bipartite-matching-dutta.cpp : Defines the entry point for the console application.
//

#include "Header.h"


int main(int argc, char* argv[])
{
//	_tmain(0, nullptr);

	BipartiteMatcher matcher;
	
	matcher.addSelf(2, 2, 8);
	
	matcher.addAlternative1(true, 2, 3, 11);
	matcher.addAlternative2(true, 2, 1, 10);
	
	matcher.addAlternative1(false, 2, 2, 5);
	matcher.addAlternative2(false, 3, 1, 6);

	matcher.displayWeights();

	matcher.solve();

	Point rSelf = matcher.getResult(0);
	printf("I should go to (%d, %d).\n", rSelf.X, rSelf.Y);
	Point rOther = matcher.getResult(1);
	printf("Other robot should go to (%d, %d).\n", rOther.X, rOther.Y);

    return 0;
}