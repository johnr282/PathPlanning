# Path Planning Algorithms
In this project, I implemented five different path planning algorithms in C++ to find the shortest path between two coordinates in an occupancy grid. The input is a simple text file with the dimensions of the grid, a grid of either 1s or 0s (0 is a walkable space, 1 is an obstacle), and two (row, column) coordinates for the start and goal locations. 

Example input file: 

```
5 5
0 0 0 1 0
0 1 0 0 1
0 0 0 0 1
1 1 0 0 0
1 1 0 0 1
0 0
3 4
```
In the above input file, there are 5 rows, 5 columns, the start location is at row 0, column 0, and the goal location is at row 3, column 4. 

The algorithms I implemented are breadth-first search, depth-first search, greedy best-first search, Dijkstra's algorithm, and A* search. For each algorithm, I output the path it finds between the start and goal coordinates (or "no path found"), the length of that path, and how many grid cells the algorithm examined. I define examining a grid cell as inserting a cell into a search container, such as a priority queue or a stack. This is a measure of how much of the occupacancy grid the algorithm had to look at while calculating the path, a rudimentary metric of the algorithm's efficiency. 

Example outputs:

![test8_output_1](https://user-images.githubusercontent.com/112778919/210175983-cc2bbe09-4fc5-4f5a-9436-600c74ba6a88.png)

![test8_output_2](https://user-images.githubusercontent.com/112778919/210176053-d0ef41c1-0875-466c-bdb3-4436062b2019.png)

![test8_output_3](https://user-images.githubusercontent.com/112778919/210176057-8dd1e148-522e-4920-acac-fe8405bf9ac9.png)
