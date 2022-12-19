#pragma once

#include <vector>
#include <queue>
#include <climits>
#include "grid_map.h"



class AStar {
private:

	struct Vertex {
		
		// Location of the preceding vertex in the path in vertices
		Coordinate prev_vertex;

		// Location of vertex in vertices
		Coordinate loc;

		// Sum of estimated cost to goal and cost from start
		int f_score = INT_MAX;

		// Cost to get from start to vertex
		int g_score = 0;

		// Stores type of vertex 
		Cell type;

	}; // class Vertex

	// Functor to compare two Vertex pointers; returns true if Vertex a's f is less
	// than Vertex b's f
	class FComp {
	public:

		bool operator()(const Vertex* a, const Vertex* b) {
			return a->f_score < b->f_score;
		}
	}; // class FComp

// ---------- Member variables ----------

	// Stores all the vertices in the map
	std::vector<std::vector<Vertex>> vertices;

	// Min f_score priority queue; Vertex with lowest f_score has highest priority
	std::priority_queue<Vertex*, std::vector<Vertex*>, FComp> open_list;

	// Finds the shortest path between these two vertices
	Coordinate start;

	Coordinate goal;

public: 

// ---------- Member functions ----------

	// Constructor
	AStar(const std::vector<std::vector<Cell>>& map_in, const Coordinate& start_in,
		const Coordinate& goal_in)
		: start{ start_in }, goal{ goal_in } {
		// Checks that start and goal are walkable spaces
		if (map_in[start.row][start.col] != Cell::walkable || map_in[goal.row][goal.col] != Cell::walkable) {
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
	void findPath() {
		
	}

private:

	// Estimates the cost to get from v to goal
	int calculateH(Vertex* v) {
		Vertex* v_goal = &(vertices[goal.row][goal.col]);
		int x_dist = abs(v_goal->loc.col - v->loc.col);
		int y_dist = abs(v_goal->loc.row - v->loc.row);
		return x_dist + y_dist;
	}




}; // class AStar