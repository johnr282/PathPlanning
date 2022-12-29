#include <iostream>
#include <vector>
#include "dijkstra.h"
#include "a_star.h"
#include "bfs_dfs.h"



// ---------- Helper function prototypes ----------


// Returns a 2D vector of cells read in from cin; input file must be in following format: 
// <num_rows> <num_cols>
// <data>
// 1 is obstacle, 0 is walkable
static std::vector<std::vector<Cell>> readMap();

// Reads in start and goal coordinate from cin; in input file, start coordinate must immediately
// follow data, and goal coordinate must immediately follow goal coordinate
static std::pair<Coordinate, Coordinate> readStartGoal();

// Check that start and goal coordinates are both walkable
void checkStartGoal(const std::vector<std::vector<Cell>>& map, const Coordinate& start, const Coordinate& goal);


using Map = std::vector<std::vector<Cell>>;

int main() {
	// Reads map data from cin or input file
	Map map = readMap();

	// Reads start and goal coordinates from cin or input file
	std::pair<Coordinate, Coordinate> path_ends = readStartGoal();

	// Sets start and goal coordinates
	Coordinate start = path_ends.first;
	Coordinate goal = path_ends.second;
	// Check that start and goal are both walkable
	checkStartGoal(map, start, goal);
	map[start.row][start.col] = Cell::start;
	map[goal.row][goal.col] = Cell::goal;

	// Prints original map
	std::cout << "\nOriginal map:\n\n";
	printMap(map);

	// Runs path planning algorithms and prints the paths they find, the length of the path, 
	// and how many cells were examined in the process (a simple measure of efficiency)

	Dijkstra d_path(map, start, goal);
	Map d_map = d_path.findPath();
	printMap(d_map);
	
	AStar a_path(map, start, goal);
	Map a_map = a_path.findPath();
	printMap(a_map);

	return 0;
} // main()



// ---------- Helper function declarations ----------


// Returns a 2D vector of cells read in from cin; input file must be in following format: 
// <num_rows> <num_cols>
// <data>
// 1 is obstacle, 0 is walkable
static std::vector<std::vector<Cell>> readMap() {
	int num_rows, num_cols;
	std::cin >> num_rows >> num_cols;
	std::vector<std::vector<Cell>> map(num_rows, std::vector<Cell>(num_cols));

	for (int i = 0; i < num_rows; ++i) {
		for (int j = 0; j < num_cols; ++j) {
			int cell_int;
			std::cin >> cell_int;
			switch (cell_int) {
			case 0:
				map[i][j] = Cell(Cell::walkable);
				break;
			case 1:
				map[i][j] = Cell(Cell::obstacle);
				break;
			default:
				std::cerr << "Error in input file\n";
				exit(1);
			} // switch(cell_int)
		} // for(j)
	} // for(i)
	return map;
} // readMap()


// Reads in start and goal coordinate from cin; in input file, start coordinate must immediately
// follow data, and goal coordinate must immediately follow goal coordinate
static std::pair<Coordinate, Coordinate> readStartGoal() {
	int x_s, y_s, x_g, y_g;
	std::cin >> x_s >> y_s >> x_g >> y_g;
	return { {x_s, y_s}, {x_g, y_g} };
}

// Check that start and goal coordinates are both walkable
void checkStartGoal(const std::vector<std::vector<Cell>>& map, const Coordinate& start, const Coordinate& goal) {
	if (map[start.row][start.col] != Cell::walkable || map[goal.row][goal.col] != Cell::walkable) {
		std::cerr << "Invalid start or goal coordinate\n";
		exit(1);
	}
}






