#include <iostream>
#include <vector>
#include "grid_map.h"
#include "dijkstra.h"
#include "a_star.h"


int main() {
	std::vector<std::vector<Cell>> map = readMap();
	std::pair<Coordinate, Coordinate> path_ends = readStartGoal();

	Dijkstra d_path(map, path_ends.first, path_ends.second);
	d_path.findPath();

	AStar a_path(map, path_ends.first, path_ends.second);
	a_path.findPath();
	return 0;
} // main()



