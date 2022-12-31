#pragma once

#include <deque>
#include "structs.h"


// Implementation of breadth first search and depth first search
class BreadthDepthSearch {
private:

	struct Vertex {

		// Location of the preceding vertex in the path
		Coordinate prev_vertex = { -1, -1 };

		// Location of vertex in vertices
		Coordinate loc;

		// Stores type of vertex
		Cell type;

		// Set to true when vertex is pushed into queue/stack
		bool visited = false;

	}; // Vertex struct	

	enum class SearchType {
		stack, queue
	};


// ---------- Member variables ----------

	// Stores data about each vertex; keeps track of which vertices have been visited
	std::vector<std::vector<Vertex>> vertices;

	// Used for printing the path
	std::vector<std::vector<Cell>> map;

	// Acts as queue in breadth first search, stack in depth first search
	std::deque<Vertex*> dq;

	// Finds the shortest path between these two vertices
	Coordinate start;

	Coordinate goal;

	// Number of vertices explored
	int num_v_explored = 0;

	// Length of path
	int total_path_length = 0;


public: 

// ---------- Member functions ----------


	// Constructor
	BreadthDepthSearch(const std::vector<std::vector<Cell>>& map_in, const Coordinate& start_in,
		const Coordinate& goal_in)
		: map{ map_in }, start{ start_in }, goal{ goal_in } {
		// Checks that start and goal are walkable spaces
		if (map_in[start.row][start.col] != Cell::start || map_in[goal.row][goal.col] != Cell::goal) {
			std::cerr << "Invalid start or goal coordinate\n";
			exit(1);
		}

		// Initializes vertices to be the same size as map_in
		size_t rows = map_in.size();
		size_t cols = map_in[0].size();
		vertices = std::vector<std::vector<Vertex>>(rows, std::vector<Vertex>(cols));

		// Gives each vertex the correct type and location
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				vertices[i][j].loc = { i, j };
				vertices[i][j].type = map_in[i][j];
			}
		}
	} // BreadthDepthSearch()


	// Uses breadth first search to find the shortest path between start and goal
	std::vector<std::vector<Cell>> findPathBFS() {
		return findPath(SearchType::queue);
	} // findPathBFS()

	
	// Uses depth first search to find the shortest path between start and goal
	std::vector<std::vector<Cell>> findPathDFS() {
		return findPath(SearchType::stack);
	} // findPathDFS()

private:

	// Helper function for findPathBFS() and findPathDFS(); 
	std::vector<std::vector<Cell>> findPath(SearchType type) {
		// Mark start vertex as visited and push it into the deque
		Vertex* start_v = &(vertices[start.row][start.col]);
		start_v->visited = true;
		dq.push_back(start_v);

		while (!dq.empty()) {
			Vertex* curr_v = nullptr;
			// If type is queue, meaning BFS, get curr_v from front of the deque; if type is 
			// stack, meaning DFS, get curr_V from the back or the deque
			switch (type) {
			case SearchType::queue:
				curr_v = dq.front();
				dq.pop_front();
				break;
			case SearchType::stack:
				curr_v = dq.back();
				dq.pop_back();
				break;
			}
			// If goal is found, break out of while loop
			if (pushAdj(curr_v)) {
				break;
			}
		} 
		// Backtrack from goal to start to find the shortest path between start and goal
		reconstructPath();
		// Print data describing path
		switch (type) {
		case SearchType::queue:
			printBFSData();
			break;
		case SearchType::stack:
			printDFSData();
			break;
		}

		return map;
	} // findPath()



	// Pushes vertices adjacent to v into deque if unvisited; returns true if goal is found, false otherwise
	// Same for both BFS and DFS
	bool pushAdj(Vertex* v) {
		// Above vertex
		// Check for out of bounds indexing
		if (v->loc.row != 0) {
			Vertex* v_up = &(vertices[v->loc.row - 1][v->loc.col]);
			pushV(v, v_up);
			// Return true if v_up is the goal
			if (v_up->loc == goal) {
				return true;
			}
		}

		// Below vertex
		if (v->loc.row != vertices.size() - 1) {
			Vertex* v_down = &(vertices[v->loc.row + 1][v->loc.col]);
			pushV(v, v_down);
			if (v_down->loc == goal) {
				return true;
			}
		}

		// Left vertex
		if (v->loc.col != 0) {
			Vertex* v_left = &(vertices[v->loc.row][v->loc.col - 1]);
			pushV(v, v_left);
			if (v_left->loc == goal) {
				return true;
			}
		}

		// Right vertex
		if (v->loc.col != vertices[0].size() - 1) {
			Vertex* v_right = &(vertices[v->loc.row][v->loc.col + 1]);
			pushV(v, v_right);
			if (v_right->loc == goal) {
				return true;
			}
		}
		// None of the adjacent vertices are the goal, so return false
		return false;
	} // pushAdj()


	// Helper function for pushAdj()
	void pushV(Vertex* v, Vertex* adj_v) {
		// If adj_v is unvisited and walkable, mark it as visited, push it into deque, 
		// and set prev_vertex as v
		if (!adj_v->visited && isWalkable(adj_v)) {
			++num_v_explored;
			adj_v->visited = true;
			dq.push_back(adj_v);
			adj_v->prev_vertex = v->loc;
		}
	} // pushV()

	// Backtrack from goal to find the shortest path between start and goal; sets the type of
	// each vertex in the path equal to "path"
	void reconstructPath() {
		Vertex* v_goal = &(vertices[goal.row][goal.col]);
		Coordinate v_path = v_goal->prev_vertex;
		++total_path_length;
		while (!(v_path == start)) {
			if (v_path == Coordinate{ -1, -1 }) {
				std::cout << "No path found\n";
				break;
			}
			map[v_path.row][v_path.col] = Cell::path;
			Vertex* v = &(vertices[v_path.row][v_path.col]);
			v_path = v->prev_vertex;
			++total_path_length;
		}
	} // reconstructPath()

	// Prints out data describing BFS path
	void printBFSData() const {
		std::cout << "Breadth-first search path \n";
		std::cout << "Cells examined: " << num_v_explored << "\n";
		std::cout << "Path length: " << total_path_length << "\n\n";
	} // printBFSData()

	// Prints out data describing DFS path
	void printDFSData() const {
		std::cout << "Depth-first search path \n";
		std::cout << "Cells examined: " << num_v_explored << "\n";
		std::cout << "Path length: " << total_path_length << "\n\n";
	} // printDFSData()

	// Returns true if a given vertex is either walkable, start, or goal
	bool isWalkable(Vertex* v) {
		return v->type == Cell::walkable || v->type == Cell::start || v->type == Cell::goal;
	} // isWalkable()

}; // BreadthDepthSearch class