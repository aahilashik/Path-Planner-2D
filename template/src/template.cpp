#include <iostream>
#include <chrono>

#include <vector>
#include <list>

#include <algorithm>

#include <math.h>

#include "../include/template.h"

using namespace std;

/*
        // Node Structure
        struct Node {
            // Coordinates/Pose of the Node
            int     x, y;

            // To store Cost and Pose Related Parameter
            map<string, double> costMap;
            map<string, bool>   nodeState;

            // Vector/List of Neighbours
            vector<Node*>   nodeNeighbours;

            // Parent Nodes of each node
            map<string, Node*> parents;
        };
*/

bool yourPPAlgo::setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) {
    // Nodes Initialization
    for (int x = 0; x < this->mapWidth; x++)
        for (int y = 0; y < this->mapHeight; y++) {
            /*  By Default

                    nodes[x*this->mapWidth + y].x = x;
                    nodes[x*this->mapWidth + y].y = y;
                    nodes[x*this->mapWidth + y].nodeState["isObstacle"] = !map2D[x][y]; // 1 - Free | 0 - Obstacle
                    nodes[x*this->mapWidth + y].nodeState["isExplored"] = false;

                                    Neighbours Positions                        Neighbours Coordinates
                    
                            neighbour   neighbour   neighbour           x0-1,y0-1   x0-1,y0+0   x0-1,y0+1
                                      \     |     /                               \     |     /
                            neighbour -- current -- neighbour           x0+0,y0-1  -- x0,y0 --  x0+0,y0+1
                                      /     |     \                               /     |     \
                            neighbour   neighbour   neighbour           x0+1,y0-1   x0+1,y0+0   x0+1,y0+1

                    // All the neighbours(x1, y2) appended to the each node(x0, y0) by following way:

                        nodes[y0*this->mapWidth + x0].nodeNeighbours.push_back(&nodes[x1*this->mapWidth + y1]);                
                        
            */

            // Use costMap member variable of map<string, double> type
            // to store cost parameters like path cost, state cost, etc.
            // Syntax: 
            //  nodes[<x> * this->mapWidth + <y>].costMap["<cost_name>"] = <value>;
            // Examples
            // 1) To store calculated path cost
                nodes[x * this->mapWidth + y].costMap["PathCost"] = 11.5;
            // 2) To set the value of state cost as inf
                nodes[x * this->mapWidth + y].costMap["StateCost"] = INFINITY;


            // Use nodeState member variable of map<string, bool> type
            // to store node status conditions like path cost, state cost, etc.
            // Syntax: 
            //  nodes[<x> * this->mapWidth + <y>].nodeState["<status_name>"] = <value>;
            // Examples
            // 1) To set the status of the node as not an obstacle
                nodes[x * this->mapWidth + y].nodeState["IsObstacle"] = false;
            // 2) To set the status of the node as explore/visited
                nodes[x * this->mapWidth + y].costMap["IsVisited"] = true;

            // To set the parent node of each node
            nodes[x * this->mapWidth + y].parents["Parent"] = nullptr;	// No parents
        }

    // Setup starting conditions
    // Set current node as start node
    Node *nodeCurrent = nodeStart;
    // Set Path and State Cost of current node as inf
    nodeCurrent->costMap["PathCost"] = INFINITY;
    nodeCurrent->costMap["StateCost"] = INFINITY;


            /*
            
                    Do the Logic here
                            &
                    Append the Planned path to the this->plannedPath(vector<Node*>)
            
            */
            cout    << "WARN : No Path Planning Algorithm is added yet!\n"
                    << "\tKindly update setPlannedPath function..!" << endl;
            return false;

    
    // Append the Plannned by backtrack the parent over child node
    if (nodeCurrent == nodeGoal) {
        cout << "INFO : Goal Reached!" << endl;
        Node *nodePath = nodeGoal;
        while (nodePath->parents["Parent"] != nullptr) {
            this->plannedPath.push_back(nodePath);
            // Backtrack the parent
            nodePath = nodePath->parents["Parent"];
        }
    } else return false; 
}