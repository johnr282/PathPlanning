#pragma once

#include <vector>
#include <limits>
#include <queue>
#include <climits>
#include "structs.h"

class Dijkstra {
private: 

	// Stores information needed for Dijkstra's algorithm for each vertex in map
	struct Vertex {
		
		// Length of the shortest path from start
		int path_length = INT_MAX;

		// Location of the preceding vertex in the shortest path from start
		Coordinate prev_vertex = { -1, -1 };

		// Stores indices of vertex in 2D vector vertices
		Coordinate loc;

		// Stores type of vertex (walkable, obstacle, etc.)
		Cell type;

		// Whether the shortest path from start is known
		bool path_known = false;
	}; // class Vertex

	// Functor to compare two Vertex pointers; returns true if Vertex a's path_length is 
	// less than Vertex b's path_length
	class PathComp {
	public: 

		bool operator()(const Vertex* a, const Vertex* b) {
			return a->path_length > b->path_length;
		}
	}; // class PathComp

// ---------- Member variables ----------

	// Stores information for Dijkstra's about each location in map
	std::vector<std::vector<Vertex>> vertices;

	// Used for printing the path
	std::vector<std::vector<Cell>> map;

	// Min path_length priority queue for Dijkstra's algorithm; Vertex with lowest path_length
	// have highest priority
	std::priority_queue<Vertex*, std::vector<Vertex*>, PathComp> pq;
	
	// Dijkstra's will find the shortest path between these two locations 
	Coordinate start;
	
	Coordinate goal;

	// Number of vertices explored
	int num_v_explored = 0;

	// Length of path
	int total_path_length = 0;

public: 

// ---------- Member functions ----------

	// Constructor
	Dijkstra(const std::vector<std::vector<Cell>>& map_in, const Coordinate& start_in,
		const Coordinate& goal_in)
		: map{ map_in }, start{ start_in }, goal{ goal_in } { 
		// Checks that both start and goal are walkable spaces
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
	} // Dijkstra()


	// Uses Dijkstra's algorithm to find the shortest path between start and goal; changes
	// type of each Vertex in that path to "path"
	std::vector<std::vector<Cell>> findPath() { 
		// Set start vertex's path_length to 0 and add it to pq
		vertices[start.row][start.col].path_length = 0;
		pq.push(&vertices[start.row][start.col]);

		while (!pq.empty()) {
			// Get vertex with smallest path_length out of the pq
			Vertex* min_v = pq.top();
			pq.pop();
			// If the shortest path from start to min_v is not known yet and min_v is walkable
			if (!min_v->path_known && isWalkable(min_v)) {
				min_v->path_known = true;
				// Update the path_length of adjacent vertices and add new vertices to pq
				updateAdj(min_v);
				++num_v_explored;
			}
		}

		// Backtrack from goal to find the shortest path between start and goal
		reconstructPath();
		// Print the map; vertices in the shortest path will be appear as an 'x'
		printData();

		return map;
	}

private:
	
	// Prints out map of vertices, an x means that vertex is part of the shortest path
	void printData() const {
		std::cout << "Dijkstra's path \n";
		std::cout << "Cells examined: " << num_v_explored << "\n";
		std::cout << "Path length: " << total_path_length << "\n\n";
	} // printData()

	// Updates the path_length of all vertices adjacent to given vertex and adds new vertices
	// to pq
	void updateAdj(Vertex* v) {
		// Calculate new path length coming from v
		int new_path_len = v->path_length + 1;
		// Update vertex above if new_path_len is less than its current path_length
		// Check if vertex above is walkable and check for indexing out of bounds
		if (v->loc.row != 0) {
			Vertex* v_up = &(vertices[v->loc.row - 1][v->loc.col]);
			++num_v_explored;
			if (isWalkable(v_up)) {
				if (new_path_len < v_up->path_length) {
					v_up->path_length = new_path_len;
					v_up->prev_vertex = v->loc;
					pq.push(v_up);
				}
			}
		}
		
		// Repeat above process vertices below, left, and right
		if (v->loc.row != vertices.size() - 1) {
			Vertex* v_down = &(vertices[v->loc.row + 1][v->loc.col]);
			++num_v_explored;
			if (isWalkable(v_down)) {
				if (new_path_len < v_down->path_length) {
					v_down->path_length = new_path_len;
					v_down->prev_vertex = v->loc;
					pq.push(v_down);
				}
			}
		}

		if (v->loc.col != vertices[0].size() - 1) {
			Vertex* v_right = &(vertices[v->loc.row][v->loc.col + 1]);
			++num_v_explored;
			if (isWalkable(v_right)) {
				if (new_path_len < v_right->path_length) {
					v_right->path_length = new_path_len;
					v_right->prev_vertex = v->loc;
					pq.push(v_right);
				}
			}
		}

		if (v->loc.col != 0) {
			Vertex* v_left = &(vertices[v->loc.row][v->loc.col - 1]);
			++num_v_explored;
			if (isWalkable(v_left)) {
				if (new_path_len < v_left->path_length) {
					v_left->path_length = new_path_len;
					v_left->prev_vertex = v->loc;
					pq.push(v_left);
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

	// Returns true if a given vertex is either walkable, start, or goal
	bool isWalkable(Vertex* v) {
		return v->type == Cell::walkable || v->type == Cell::start || v->type == Cell::goal;
	} // isWalkable()


}; // class Dijkstra