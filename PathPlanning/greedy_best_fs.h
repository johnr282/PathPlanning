#pragma once

#include <queue>
#include <climits>
#include "structs.h"


class GreedyBestFS {
private:

	struct Vertex {

		// Location of the preceding vertex in the path in vertices
		Coordinate prev_vertex = { -1, -1 };

		// Location of vertex in vertices
		Coordinate loc;

		// Estimate of the distance to the goal
		int h_score = INT_MAX;

		// Stores type of vertex 
		Cell type;

		// Set to true when vertex is placed into open list
		bool in_open = false;

	}; // Vertex struct


	// Functor to compare two Vertex pointers; returns true if Vertex a's h is greater
	// than Vertex b's h
	class HComp {
	public:

		bool operator()(const Vertex* a, const Vertex* b) {
			return a->h_score > b->h_score;
		}
	}; // class HComp


// ---------- Member variables ----------

	// Stores all the vertices in the map
	std::vector<std::vector<Vertex>> vertices;

	// Used for printing the path
	std::vector<std::vector<Cell>> map;

	// Min h_score priority queue containing vertices that have been visited
	std::priority_queue<Vertex*, std::vector<Vertex*>, HComp> open_list;

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
	GreedyBestFS(const std::vector<std::vector<Cell>>& map_in, const Coordinate& start_in,
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
	} // GreedyBestFS()


	// Uses greedy best-first search algorithm to find the shortest path between start and goal
	std::vector<std::vector<Cell>> findPath() {
		// Insert start vertex into open list
		Vertex* start_v = &(vertices[start.row][start.col]);
		open_list.push(start_v);
		start_v->in_open = true;

		while (!open_list.empty()) {
			// Get vertex with minimum h_score out of open_list
			Vertex* curr_v = open_list.top();
			open_list.pop();
			// If curr_v is the goal, break out of the while loop
			if (curr_v->loc == goal) {
				break;
			}
			// Update adjacent vertices h_scores and push them into open_list
			updateAdj(curr_v);
		}
		// Backtrack from goal to start to find the shortest path between start and goal
		reconstructPath();
		// Print data describing path
		printData();

		return map;
	} // findPath()

private:

	// Estimates the cost to get from v to goal
	int calculateH(Vertex* v) {
		Vertex* v_goal = &(vertices[goal.row][goal.col]);
		int x_dist = abs(v_goal->loc.col - v->loc.col);
		int y_dist = abs(v_goal->loc.row - v->loc.row);
		return x_dist + y_dist;
	} // calculateH()


	void updateAdj(Vertex* v) {
		// Above vertex
		// Check for out of bounds indexing
		if (v->loc.row != 0) {
			Vertex* v_up = &(vertices[v->loc.row - 1][v->loc.col]);
			// Calculates v_up's h_score and pushes it into open_list
			updateV(v, v_up);
		}

		// Below vertex
		if (v->loc.row != vertices.size() - 1) {
			Vertex* v_down = &(vertices[v->loc.row + 1][v->loc.col]);
			updateV(v, v_down);
		}

		// Left vertex
		if (v->loc.col != 0) {
			Vertex* v_left = &(vertices[v->loc.row][v->loc.col - 1]);
			updateV(v, v_left);
		}

		// Right vertex
		if (v->loc.col != vertices[0].size() - 1) {
			Vertex* v_right = &(vertices[v->loc.row][v->loc.col + 1]);
			updateV(v, v_right);
		}
	} // updateAdj()


	// Helper function for updateAdj()
	void updateV(Vertex* src_v, Vertex* adj_v) {
		// Check if adj_v is walkable and not in open_list already
		if (isWalkable(adj_v) && !adj_v->in_open) {
			++num_v_explored;
			// Calculate adj_v's h_score, push it into open_list, and set its prev_vertex
			// to src_v
			adj_v->h_score = calculateH(adj_v);
			open_list.push(adj_v);
			adj_v->in_open = true;
			adj_v->prev_vertex = src_v->loc;
		}
	}


	// Returns true if a given vertex is either walkable, start, or goal
	bool isWalkable(Vertex* v) {
		return v->type == Cell::walkable || v->type == Cell::start || v->type == Cell::goal;
	}


	// Prints out data describing path
	void printData() const {
		std::cout << "Greedy best-first search path\n";
		std::cout << "Cells examined: " << num_v_explored << "\n";
		std::cout << "Path length: " << total_path_length << "\n\n";
	} // printData()


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
	 


}; // GreedyBestFS class