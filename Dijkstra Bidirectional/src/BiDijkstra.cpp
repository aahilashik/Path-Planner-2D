#include <iostream>
#include <chrono>
#include <thread>

#include <vector>
#include <list>

#include <algorithm>

#include <math.h>

#include "../include/BiDijkstra.h"

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

bool BiDijkstra::setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) {
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
                    if (this->isBidirectional) {
                        nodes[x*this->mapWidth + y].nodeState["isStartExplored"] = false;
                        nodes[x*this->mapWidth + y].nodeState["isGoalExplored"] = false;
                    }
                    else
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
            //      nodes[<x> * this->mapWidth + <y>].costMap["<cost_name>"] = <value>;
            // Examples
            // 1) To store calculated path cost
            //     nodes[x * this->mapWidth + y].costMap["PathCost"] = 11.5;
            // 2) To set the value of state cost as inf
            //     nodes[x * this->mapWidth + y].costMap["StateCost"] = INFINITY;

            // Since its bidirectional we need to compute cost from start and goal node
            nodes[x * this->mapWidth + y].costMap["StartStepCost"] = INFINITY;
            nodes[x * this->mapWidth + y].costMap["GoalStepCost"] = INFINITY;

            // Use nodeState member variable of map<string, bool> type
            // to store node status conditions like path cost, state cost, etc.
            // Syntax: 
            //  nodes[<x> * this->mapWidth + <y>].nodeState["<status_name>"] = <value>;
            // Examples
            // 1) To set the status of the node as not an obstacle
            //     nodes[x * this->mapWidth + y].nodeState["IsObstacle"] = false;
            // 2) To set the status of the node as explore/visited
            //     nodes[x * this->mapWidth + y].nodeState["IsVisited"] = true;
            nodes[x * this->mapWidth + y].nodeState["isStartExplored"] = false;
            nodes[x * this->mapWidth + y].nodeState["isGoalExplored"] = false;

            // To set the parent node of each node
            nodes[x * this->mapWidth + y].parents["StartParent"] = nullptr;
            nodes[x * this->mapWidth + y].parents["GoalParent"] = nullptr;
        }

            /*           
                                        Do the Logic here
                                                &
                    Append the Planned path to the this->plannedPath(vector<Node*>)
            */

    // Setup starting conditions
    // Set current node as start node
    Node *nodeCurrent = nodeStart;
    // Set Start and Goal Step Cost of start and goal node as 0.0
    nodeStart->costMap["StartStepCost"] = 0.0f;
    nodeStart->costMap["GoalStepCost"] = 0.0f;
    nodeGoal->costMap["StartStepCost"] = 0.0f;
    nodeGoal->costMap["GoalStepCost"] = 0.0f;

    openNodesList[0].clear();
    openNodesList[1].clear();
    openNodesList[0].push_back(nodeStart);
    openNodesList[1].push_back(nodeGoal);

    int iteration = 0;
    while (!openNodesList[0].empty() && !openNodesList[1].empty()) {
        iteration++;

        // Check whether the current is intersecting with both direction
        if (nodeCurrent->nodeState["isStartExplored"] && nodeCurrent->nodeState["isGoalExplored"]) {
            this->nodeMiddle = nodeCurrent;
            break;
        }
        
        // Iterate the process two times for Start and Goal Direction
        // dir = 0 -> Search from Start Pose 
        // dir = 1 -> Search from Goal Pose 
        for (int dir=0; dir<2; dir++) {
            // Sort the open list in ascending order with respect to step cost
            openNodesList[dir].sort([&dir](Node* lhs, Node* rhs){ 
                double lhsCost = lhs->costMap[(dir==0?"StartStepCost":"GoalStepCost")]; 
                double rhsCost = rhs->costMap[(dir==0?"StartStepCost":"GoalStepCost")];
                return lhsCost < rhsCost; 
            } );
            
            while(!openNodesList[dir].empty() && openNodesList[dir].front()->nodeState[(dir==0?"isStartExplored":"isGoalExplored")])
                openNodesList[dir].pop_front();

            if (openNodesList[dir].empty())
                break;

            nodeCurrent = openNodesList[dir].front();
            nodeCurrent->nodeState[(dir==0?"isStartExplored":"isGoalExplored")] = true; // We only explore a node once

            for (auto nodeNeighbour : nodeCurrent->nodeNeighbours) {
                if (!nodeNeighbour->nodeState[(dir==0?"isStartExplored":"isGoalExplored")] && !nodeNeighbour->nodeState["isObstacle"]) 
                    openNodesList[dir].push_back(nodeNeighbour);

                float fPossiblyLowerGoal = nodeCurrent->costMap[(dir==0?"StartStepCost":"GoalStepCost")] + distance(nodeCurrent, nodeNeighbour);

                if (fPossiblyLowerGoal < nodeNeighbour->costMap[(dir==0?"StartStepCost":"GoalStepCost")]) {
                    nodeNeighbour->parents[(dir==0?"StartParent":"GoalParent")] = nodeCurrent;
                    nodeNeighbour->costMap[(dir==0?"StartStepCost":"GoalStepCost")] = fPossiblyLowerGoal;
                }
            }	
        }

        if (true) {                 // Animation
            system("clear");
            this->printMap();
            this_thread::sleep_for(chrono::milliseconds(100));
        }
        
    }
    cout << "\n Iteration : " << iteration << endl;
    // Append the Plannned by backtrack the parent over child node
    if (nodeMiddle != nullptr) {
        cout << "INFO : Goal Reached!" << endl;
        this->plannedPath.push_back(nodeMiddle);
        Node *nodeSP = this->nodeMiddle->parents["StartParent"];
        Node *nodeGP = this->nodeMiddle->parents["GoalParent"];
        while (nodeSP != nullptr) {
                this->plannedPath.push_front(nodeSP);
                // Backtrack the parent from start
                nodeSP = nodeSP->parents["StartParent"];
            }
        while (nodeGP != nullptr) {
                this->plannedPath.push_back(nodeGP);
                // Backtrack the parent from goal
                nodeGP = nodeGP->parents["GoalParent"];;
            }

    } else return false; 
}