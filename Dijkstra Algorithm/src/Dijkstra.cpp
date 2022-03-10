#include <iostream>
#include <chrono>
#include <thread>

#include <vector>
#include <list>

#include <algorithm>

#include <math.h>

#include "../include/Dijkstra.h"

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

bool Dijkstra::setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) {
    // Cost Lambdas Functions
    auto distance = [](Node* a, Node* b) {
        return sqrtf(pow((a->x - b->x), 2) + pow((a->y - b->y), 2));
    };

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
            //     nodes[x * this->mapWidth + y].costMap["PathCost"] = 11.5;
            // 2) To set the value of state cost as inf
            //     nodes[x * this->mapWidth + y].costMap["StateCost"] = INFINITY;
            nodes[x * this->mapWidth + y].costMap["StepCost"] = INFINITY;

            // Use nodeState member variable of map<string, bool> type
            // to store node status conditions like path cost, state cost, etc.
            // Syntax: 
            //  nodes[<x> * this->mapWidth + <y>].nodeState["<status_name>"] = <value>;
            // Examples
            // 1) To set the status of the node as not an obstacle
            //     nodes[x * this->mapWidth + y].nodeState["IsObstacle"] = false;
            // 2) To set the status of the node as explore/visited
            //     nodes[x * this->mapWidth + y].nodeState["IsVisited"] = true;
            nodes[x * this->mapWidth + y].nodeState["isExplored"] = false;

            // To set the parent node of each node
            nodes[x * this->mapWidth + y].parents["Parent"] = nullptr;	// No parents
        }

    // Setup starting conditions
    // Set current node as start node
    Node *nodeCurrent = nodeStart;
    // Set Step Cost of current node as 0.0
    nodeCurrent->costMap["StepCost"] = 0.0f;

            /*           
                                        Do the Logic here
                                                &
                    Append the Planned path to the this->plannedPath(vector<Node*>)
            */

    openNodesList.clear();
    openNodesList.push_back(nodeStart);

    int iteration = 0;
    while (!openNodesList.empty() && nodeCurrent != nodeGoal) {
        iteration++;
        openNodesList.sort([](Node* lhs, Node* rhs){ return lhs->costMap["StepCost"] < rhs->costMap["StepCost"]; } );
        
        while(!openNodesList.empty() && openNodesList.front()->nodeState["isExplored"])
            openNodesList.pop_front();

        if (openNodesList.empty())
            break;

        nodeCurrent = openNodesList.front();
        nodeCurrent->nodeState["isExplored"] = true; // We only explore a node once
        
        for (auto nodeNeighbour : nodeCurrent->nodeNeighbours) {
            if (!nodeNeighbour->nodeState["isExplored"] && !nodeNeighbour->nodeState["isObstacle"])
                openNodesList.push_back(nodeNeighbour);

            float fPossiblyLowerGoal = nodeCurrent->costMap["StepCost"] + distance(nodeCurrent, nodeNeighbour);

            if (fPossiblyLowerGoal < nodeNeighbour->costMap["StepCost"]) {
                nodeNeighbour->parents["Parent"] = nodeCurrent;
                nodeNeighbour->costMap["StepCost"] = fPossiblyLowerGoal;
            }
        }	

        if (true) {                 // Animation
            system("clear");
            this->printMap();
            this_thread::sleep_for(chrono::milliseconds(100));
        }
        
    }
    cout << "\n Iteration : "<< iteration << endl;
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