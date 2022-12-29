#pragma once

#include <vector>
#include <iostream>

// Contains data structures and helper functions used in every path planning algorithm


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

// Prints given 2D vector of cells to cout
static void printMap(std::vector<std::vector<Cell>>& map) {
	for (int i = 0; i < map.size(); ++i) {
		for (int j = 0; j < map[0].size(); ++j) {
			switch (map[i][j]) {
			case Cell::obstacle:
				std::cout << "1  ";
				break;
			case Cell::walkable:
				std::cout << "0  ";
				break;
			case Cell::path:
				std::cout << RED "x  " << RESET;
				break;
			case Cell::start:
				std::cout << RED << "s  " << RESET;
				break;
			case Cell::goal:
				std::cout << RED << "g  " << RESET;
				break;
			} // switch(map[i][j]
		} // for(j)
		std::cout << "\n";
	} // for(i)
	std::cout << "\n\n";
} // printMap()
