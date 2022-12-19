#include <iostream>
#include <vector>
#include "grid_map.h"
#include "dijkstra.h"


int main() {
	std::vector<std::vector<Cell>> map = readMap();
	std::pair<Coordinate, Coordinate> path_ends = readStartGoal();
	Dijkstra path(map, path_ends.first, path_ends.second);
	path.findPath();
	return 0;
} // main()



