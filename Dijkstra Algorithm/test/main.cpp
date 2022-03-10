#include <iostream>
#include <chrono>

#include <vector>

#include "../src/Dijkstra.cpp"

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

        Dijkstra dijkstra = Dijkstra(map2D);
        
        // Set the Start Pose(x, y)
        dijkstra.setStartNode(0, 0);
        // Set the Goal Pose(x, y)
        dijkstra.setGoalNode(15, 15);
        // Start to Plan
        dijkstra.startPlan();

        // Vector to store the planned path 
        vector<PathPlanner2D::Node*> path;
        dijkstra.getPlannedPath(path);
    
        // Print the Map with planned path in terminal
        dijkstra.printMap(path);

        // // Iterate over path and print their coordinates
        // cout << "Path >> " << endl;
        // for (auto p: path) {
        //     map2D[p->x][p->y] = -1;
        //     cout << p->x << "," << p->y << " -> ";
        // }
        // cout  << endl;

}

// g++ -I /home/ubuntussd/cpp_ws/Robotics/01_COBOT_ARM/002_Path_Planner/PathPlanner/src/  01_a_star.cpp -o code && ./code