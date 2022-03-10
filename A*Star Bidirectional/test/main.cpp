#include <iostream>
#include <chrono>

#include <vector>
#include <list>

#include <algorithm>

#include <math.h>

#include "../src/BiAStar.cpp"

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

        BiAStar biAStar = BiAStar(map2D);

        // Set the Start Pose(x, y)
        biAStar.setStartNode(0, 0);
        // Set the Goal Pose(x, y)
        biAStar.setGoalNode(15, 15);
        // Start to Plan
        biAStar.startPlan();

        // Vector to store the planned path 
        vector<PathPlanner2D::Node*> path;
        biAStar.getPlannedPath(path);

        biAStar.printMap(path);

        // cout << "Path >> " << endl;
        // for (auto p: path) {
        //     map2D[p->x][p->y] = -1;
        //     cout << p->x << "," << p->y << " -> ";
        // }
        // cout  << endl;

}