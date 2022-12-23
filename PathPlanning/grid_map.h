#pragma once

#include <vector>
#include <iostream>

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

enum class Cell {
	obstacle,
	walkable,
	unknown,
	path, 
	start, 
	goal
};

struct Coordinate {
	int row, col;

	bool operator==(const Coordinate& rhs) const {
		return (this->row == rhs.row) && (this->col == rhs.col);
	}
};



// Returns a 2D vector of cells read in from cin; input file must be in following format: 
// <num_rows> <num_cols>
// <data>
// 1 is obstacle, 0 is walkable, -1 is unknown
static std::vector<std::vector<Cell>> readMap() {
	int num_rows, num_cols;
	std::cin >> num_rows >> num_cols;
	std::vector<std::vector<Cell>> map(num_rows, std::vector<Cell>(num_cols));

	for (int i = 0; i < num_rows; ++i) {
		for (int j = 0; j < num_cols; ++j) {
			int cell_int;
			std::cin >> cell_int;
			switch (cell_int) {
			case -1:
				map[i][j] = Cell(Cell::unknown);
				break;
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

// Prints given 2D vector of cells to cout
static void printMap(std::vector<std::vector<Cell>>& map, 
	const Coordinate& start, const Coordinate& goal) {
	map[start.row][start.col] = Cell::start;
	map[goal.row][goal.col] = Cell::goal;
	for (int i = 0; i < map.size(); ++i) {
		for (int j = 0; j < map[0].size(); ++j) {

			switch (map[i][j]) {
			case Cell::obstacle:
				std::cout << "1  ";
				break;
			case Cell::walkable:
				std::cout << "0  ";
				break;
			case Cell::unknown:
				std::cout << "-1 ";
				break;
			case Cell::path:
				std::cout << "x  ";
				break;
			case Cell::start:
				std::cout << "s  ";
				break;
			case Cell::goal:
				std::cout << "g  ";
				break;
			} // switch(map[i][j]
		} // for(j)
		std::cout << "\n";
	} // for(i)
} // printMap()