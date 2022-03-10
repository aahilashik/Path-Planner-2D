/*************************************************************************
	> File Name: PathPlanner.cpp
	> Author: Muhammath Ashik Farhan S
	> Mail: aahilashik@gmail.com
	> Created on: Jan 2022
	> Description: Base Code 2D Grid Based Path Planner
************************************************************************/

#include <iostream>
#include <chrono>

#include <vector>
#include <list>
#include <map>

#include <algorithm>

#include <math.h>

#include "../include/PathPlanner.h"

using namespace std;

/* -------- Constructors -------- */

PathPlanner2D::PathPlanner2D() {}

template <size_t width, size_t height>
PathPlanner2D::PathPlanner2D(int (&map2D)[width][height]) {
    this->updateMap(map2D);
}

/* -------- Member Functions -------- */


void PathPlanner2D::setBidirectionalFlag(bool flag) {
    this->isBidirectional = flag;
}

template <size_t width, size_t height>
void PathPlanner2D::updateMap(int (&map2D)[width][height]) {
    this->mapWidth  = width;
    this->mapHeight = height;

    this->nodes = new Node[width * height];

    cout << "INFO : 2D Map of " << this->mapWidth << " x " << this->mapHeight << endl;

    // Assign Default Initial Values
    for (int x=0; x<this->mapWidth; x++)
        for (int y=0; y<this->mapHeight; y++) {
            this->nodes[x*this->mapWidth + y].x = x;
            this->nodes[x*this->mapWidth + y].y = y;
            this->nodes[x*this->mapWidth + y].nodeState["isObstacle"] = !map2D[x][y];
            if (this->isBidirectional){
                this->nodes[x*this->mapWidth + y].nodeState["isStartExplored"] = false;
                this->nodes[x*this->mapWidth + y].nodeState["isGoalExplored"] = false;
            }
            else
                this->nodes[x*this->mapWidth + y].nodeState["isExplored"] = false;

            for (int _x=-1; _x<=+1; _x++) {
                for (int _y=-1; _y<=+1; _y++) {
                    // Current/Search Pose
                    if ((_x==0) & (_y==0)) 
                        continue;                              
                    // Wall
                    else if ((_x+x<0) | (_x+x>=this->mapWidth) | (_y+y<0) | (_y+y>=this->mapHeight))
                        continue;
                    // Possible Successor
                    else 
                        this->nodes[x*this->mapWidth + y].nodeNeighbours.push_back(&nodes[(x + _x) * this->mapWidth + (y + _y)]);
                }
            }

        }
}

void PathPlanner2D::setStartNode(int startX, int startY) {
    // If map is empty/null 
    if (this->nodes == nullptr) {
        cout << "WARN : Unknown Environment! Kindly update the map.." << endl;
        return;
    } 

    // Setup the Start Node
    this->nodeStart = &(this->nodes)[startX * mapWidth + startY];

    // If provided Start Node is obstacle
    if (this->nodeStart->nodeState["isObstacle"]==true) {
        cout << "WARN : Given Start Pose is Invalid/Obstacle!" << endl;
        return;
    } else 
        cout << "INFO : Start Position set successfully!" << endl;
}

void PathPlanner2D::setGoalNode(int goalX, int goalY) {
    // If map is empty/null 
    if (this->nodes == nullptr) {
        cout << "WARN : Unknown Environment! Kindly update the map.." << endl;
        return;
    }

    // Setup the Goal Node
    this->nodeGoal = &(this->nodes)[goalX * mapWidth + goalY];

    // If provided Goal Node is obstacle
    if ((this->nodeGoal->nodeState["isObstacle"]==true) || (goalX*mapWidth + goalY >= mapWidth*mapHeight)) {
        cout << "WARN : Given Goal Pose is Invalid/Obstacle!" << endl;
        return;
    } else 
        cout << "INFO : Goal Position set successfully!" << endl;
}

bool PathPlanner2D::startPlan() {
    // Reset the Nodes to explore
    this->resetNodes();

    auto _begin = chrono::high_resolution_clock::now();

        if (this->nodes == nullptr) {
            cout << "WARN : Unknown Environment! Kindly update the map.." << endl;
            return false;
        }
        if (this->nodeStart == nullptr) {
            cout << "WARN : Unknown Initial Pose! Kindly set the start location..." << endl;
            return false;
        }
        if (this->nodeGoal == nullptr) {
            cout << "WARN : Unknown Goal Pose! Kindly set the goal location..." << endl;
            return false;
        }

    if (!this->setPlannedPath(nodes, nodeStart, nodeGoal))
        return false;

    auto _end = chrono::high_resolution_clock::now();
    chrono::duration<double> _elapsed = _end - _begin;
    cout << "INFO : Elapsed Time : " << _elapsed.count() << " seconds!" << endl;

}

void PathPlanner2D::getPlannedPath(vector<Node*>& path) {
    path.assign(this->plannedPath.begin(), this->plannedPath.end());
    return ;    
}

bool PathPlanner2D::setPlannedPath(Node *nodes, Node *Start, Node *Goal) {
    cout << "WARN : No Path Planning Algorithm is added yet!\n"
        << "\tKindly update setPlannedPath function..!" << endl;
    return false;
}

void PathPlanner2D::printMap(vector<Node*> path = {}) {
    // Print the Map
    for (int y=0; y<this->mapHeight; y++)
        cout << "--";
    cout << "---\n";
    for (int x=0; x<this->mapWidth; x++) {
        for (int y=0; y<this->mapHeight; y++) {
            cout << ( y==0 ? "| " : "" );
            
                if (&nodes[x * this->mapWidth + y] == nodeStart)
                    cout << "S ";
                else if (&nodes[x * this->mapWidth + y] == nodeGoal)
                    cout << "G ";
                else if (&nodes[x * this->mapWidth + y] == nodeMiddle)
                    cout << "M ";
                else if (nodes[x * this->mapWidth + y].nodeState["isObstacle"]) 
                    cout << "8 ";
                else if (std::count(path.begin(), path.end(), &nodes[x * this->mapWidth + y])) 
                    cout << "+ ";
                else if (nodes[x * this->mapWidth + y].nodeState["isStartExplored"]) 
                    cout << "` ";
                else if (nodes[x * this->mapWidth + y].nodeState["isGoalExplored"]) 
                    cout << "' ";
                else if (nodes[x * this->mapWidth + y].nodeState["isExplored"]) 
                    cout << "` ";
                else
                    cout << "  ";

            cout << ( y==this->mapHeight -1 ? "|\n" : "" );   
        }
    }
    for (int y=0; y<this->mapHeight; y++)
        cout << "--";
    cout << "---\n";
}

void PathPlanner2D::resetNodes() {

    cout << "INFO : Nodes are resetted and Path is cleared!" << endl;

    // Assign Default Initial Values
    for (int x=0; x<this->mapWidth; x++)
        for (int y=0; y<this->mapHeight; y++) {
            if (this->isBidirectional){
                this->nodes[x*this->mapWidth + y].nodeState["isStartExplored"] = false;
                this->nodes[x*this->mapWidth + y].nodeState["isGoalExplored"] = false;
            } else
                this->nodes[x*this->mapWidth + y].nodeState["isExplored"] = false;
        }
    this->plannedPath.clear();
}


/*
int main() {

    // Map Dimension
    const int width = 10, height = 10;

    // 2D Map Array 
    int map2D[width][height] =  {   {1, 0, 1, 1, 1, 1, 1, 1, 1, 1},
                                    {1, 0, 1, 1, 1, 1, 0, 1, 1, 1},
                                    {1, 0, 1, 0, 1, 1, 0, 1, 1, 1},
                                    {1, 0, 1, 0, 1, 1, 0, 0, 0, 0},
                                    {1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                    {1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                                    {1, 0, 1, 1, 1, 1, 1, 1, 0, 1},
                                    {1, 0, 1, 0, 0, 0, 0, 0, 0, 1},
                                    {1, 0, 1, 1, 1, 1, 1, 1, 1, 1}  };

    auto _begin = chrono::high_resolution_clock::now();

    PathPlanner2D pp = PathPlanner2D(map2D);
    pp.setStartNode(2, 2); 
    // pp.setGoalNode(0, 0);
    pp.startPlan();


    auto _end = chrono::high_resolution_clock::now();
    chrono::duration<double> _elapsed = _end - _begin;
    cout << "INFO : Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
}
*/