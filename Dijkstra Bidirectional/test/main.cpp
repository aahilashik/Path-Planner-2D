#include <iostream>
#include <chrono>

#include <vector>

#include "../src/BiDijkstra.cpp"

using namespace std;

// Map Dimension
const int mapWidth      = 16;
const int mapHeight     = 16;

// 2D Map Array 
int map2D[mapWidth][mapHeight] =    {   {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, };

int main() {
    BiDijkstra biDijkstra = BiDijkstra(map2D);
    
    // Set the Start Pose(x, y)
    biDijkstra.setStartNode(0, 0);
    // Set the Goal Pose(x, y)
    biDijkstra.setGoalNode(15, 15);
    // Start to Plan
    biDijkstra.startPlan();

    // Vector to store the planned path 
    vector<PathPlanner2D::Node*> path;
    biDijkstra.getPlannedPath(path);

    // Print the Map with planned path in terminal
    biDijkstra.printMap(path);

    // // Iterate over path and print their coordinates
    // cout << "Path >> ";
    // for (auto p: path) {
    //     map2D[p->x][p->y] = -1;
    //     cout << p->x << "," << p->y << " -> ";
    // }
    // cout  << endl;

}