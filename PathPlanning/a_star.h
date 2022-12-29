#pragma once

#include <vector>
#include <queue>
#include <climits>
#include <set>
#include "structs.h"


class AStar {
private:

	struct Vertex {

		// Location of the preceding vertex in the path in vertices
		Coordinate prev_vertex = { -1, -1 };

		// Location of vertex in vertices
		Coordinate loc;

		// Sum of estimated cost to goal and cost from start
		int f_score = INT_MAX;

		// Cost to get from start to vertex
		int g_score = INT_MAX;

		// Stores type of vertex 
		Cell type;

		// Set to true when vertex is explored and placed in open_list
		bool in_open = false;

	}; // class Vertex


	// Functor to compare two Vertex pointers; returns true if Vertex a's f is less
	// than Vertex b's f
	class FComp {
	public:

		bool operator()(const Vertex* a, const Vertex* b) {
			return a->f_score > b->f_score;
		}
	}; // class FComp


	// ---------- Member variables ----------

	// Stores all the vertices in the map
	std::vector<std::vector<Vertex>> vertices;

	// Used for printing the path
	std::vector<std::vector<Cell>> map;

	// Min f_score priority queue; vertex with lowest f_score has highest priority; 
	// contains vertices that still need to be explored
	std::priority_queue<Vertex*, std::vector<Vertex*>, FComp> open_list;

	// Contains vertices that have been explored already
	std::set<Vertex*> closed_list;

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
	AStar(const std::vector<std::vector<Cell>>& map_in, const Coordinate& start_in,
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
	} // AStar()

	// Uses A* to find the shortest path between start and goal
	std::vector<std::vector<Cell>> findPath() {
		// Calculate start's f_score and add it to open_list
		Vertex* v_start = &(vertices[start.row][start.col]);
		v_start->g_score = 0;
		v_start->f_score = calculateH(v_start);
		open_list.push(v_start);
		v_start->in_open = true;

		while (!open_list.empty()) {
			// Get vertex with lowest f_score out of open_list
			Vertex* v_min = open_list.top();
			open_list.pop();
			v_min->in_open = false;
			// If v_min is already in the closed_list, meaning v_min is a duplicate of 
			// a vertex that has already been explored, move on to the next v_min
			if (closed_list.find(v_min) != closed_list.end()) {
				continue;
			}
			// Put v_min in closed list
			closed_list.insert(v_min);

			// If v_min is the goal, we have found the shortest path between start and goal
			if (v_min->loc == goal) {
				break;
			}
			// Process min_v's adjacent vertices; calculate their f_scores and add them to 
			// open_list
			updateAdj(v_min);
			++num_v_explored;
		}

		// Backtrack from goal to start to find the shortest path between start and goal
		reconstructPath();
		// Print the map; vertices in the shortest path will be appear as an 'x'
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
		// New g_score for each adjacent vertex
		int new_g_score = v->g_score + 1;

		// Above vertex
		// Check for out of bounds indexing
		if (v->loc.row != 0) {
			Vertex* v_up = &(vertices[v->loc.row - 1][v->loc.col]);
			auto it = closed_list.find(v_up);
			++num_v_explored;
			// If v_up is walkable and not in closed_list
			if (isWalkable(v_up) && it == closed_list.end()) {
				// If new g_score is shorter than v_up's current g_score or v_up is
				// not in the open_list
				if (new_g_score < v_up->g_score || !v_up->in_open) {
					// Update v_up's g_score, f_score, and prev_vertex
					v_up->g_score = new_g_score;
					v_up->f_score = new_g_score + calculateH(v_up);
					v_up->prev_vertex = v->loc;
					// Add v_up to open_list; even if v_up was already in open_list, we
					// need to add it again to take the updated f_score into account
					open_list.push(v_up);
					v_up->in_open = true;
				}
			}
		}

		// Repeat process for below, left, and right vertices
		// Below vertex
		if (v->loc.row != vertices.size() - 1) {
			Vertex* v_down = &(vertices[v->loc.row + 1][v->loc.col]);
			auto it = closed_list.find(v_down);
			if (isWalkable(v_down) && it == closed_list.end()) {
				if (new_g_score < v_down->g_score || !v_down->in_open) {
					v_down->g_score = new_g_score;
					v_down->f_score = new_g_score + calculateH(v_down);
					v_down->prev_vertex = v->loc;
					open_list.push(v_down);
					v_down->in_open = true;
				}
			}
		}

		// Left vertex
		if (v->loc.col != 0) {
			Vertex* v_left = &(vertices[v->loc.row][v->loc.col - 1]);
			auto it = closed_list.find(v_left);
			++num_v_explored;
			if (isWalkable(v_left) && it == closed_list.end()) {
				if (new_g_score < v_left->g_score || !v_left->in_open) {
					v_left->g_score = new_g_score;
					v_left->f_score = new_g_score + calculateH(v_left);
					v_left->prev_vertex = v->loc;
					open_list.push(v_left);
					v_left->in_open = true;
				}
			}
		}

		// Right vertex
		if (v->loc.col != vertices[0].size() - 1) {
			Vertex* v_right = &(vertices[v->loc.row][v->loc.col + 1]);
			auto it = closed_list.find(v_right);
			++num_v_explored;
			if (isWalkable(v_right) && it == closed_list.end()) {
				if (new_g_score < v_right->g_score || !v_right->in_open) {
					v_right->g_score = new_g_score;
					v_right->f_score = new_g_score + calculateH(v_right);
					v_right->prev_vertex = v->loc;
					open_list.push(v_right);
					v_right->in_open = true;
				}
			}
		}
	} // updateAdj()

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

	// Prints out map of vertices, an x means that vertex is part of the shortest path
	void printData() const {
		std::cout << "A* path \n";
		std::cout << "Cells examined: " << num_v_explored << "\n";
		std::cout << "Path length: " << total_path_length << "\n\n";
	} // printData()

	// Returns true if a given vertex is either walkable, start, or goal
	bool isWalkable(Vertex* v) {
		return v->type == Cell::walkable || v->type == Cell::start || v->type == Cell::goal;
	}


}; // class AStar