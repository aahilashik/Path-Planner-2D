#include <iostream>
#include <chrono>
#include <thread>

#include <vector>
#include <list>

#include <algorithm>

#include <math.h>

#include "../include/AStar.h"

using namespace std;

bool AStar::setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) {
    // Cost Lambdas Functions
    auto distance = [](Node* a, Node* b) {
        return sqrtf(pow((a->x - b->x), 2) + pow((a->y - b->y), 2));
    };
    auto heuristic = [distance](Node* a, Node* b) {
        return distance(a, b);
    };

    // Nodes Initialization
    for (int x = 0; x < this->mapWidth; x++)
        for (int y = 0; y < this->mapHeight; y++) {
            nodes[x * this->mapWidth + y].costMap["PathCost"] = INFINITY;
            nodes[x * this->mapWidth + y].costMap["TotalCost"] = INFINITY;

            nodes[x * this->mapWidth + y].parents["Parent"] = nullptr;	// No parents
        }

    // Setup starting conditions
    Node *nodeCurrent = nodeStart;
    nodeStart->costMap["PathCost"] = 0.0f;
    nodeStart->costMap["TotalCost"] = heuristic(nodeStart, nodeGoal);


    openNodesList.clear();
    openNodesList.push_back(nodeStart);

    int iteration = 0;
    while (!openNodesList.empty() && nodeCurrent != nodeGoal) {
        iteration++;
        openNodesList.sort([](Node* lhs, Node* rhs){ return lhs->costMap["TotalCost"] < rhs->costMap["TotalCost"]; } );
        
        while(!openNodesList.empty() && openNodesList.front()->nodeState["isExplored"])
            openNodesList.pop_front();

        if (openNodesList.empty())
            break;


        nodeCurrent = openNodesList.front();
        nodeCurrent->nodeState["isExplored"] = true; // We only explore a node once
        
        for (auto nodeNeighbour : nodeCurrent->nodeNeighbours) {
            // cout << "Ob : " << nodeNeighbour->nodeState["isObstacle"] << endl;
            if (!nodeNeighbour->nodeState["isExplored"] && !nodeNeighbour->nodeState["isObstacle"])
                openNodesList.push_back(nodeNeighbour);

            float fPossiblyLowerGoal = nodeCurrent->costMap["PathCost"] + distance(nodeCurrent, nodeNeighbour);

            if (fPossiblyLowerGoal < nodeNeighbour->costMap["PathCost"]) {
                nodeNeighbour->parents["Parent"] = nodeCurrent;
                nodeNeighbour->costMap["PathCost"] = fPossiblyLowerGoal;
                nodeNeighbour->costMap["TotalCost"] = nodeNeighbour->costMap["PathCost"] + heuristic(nodeNeighbour, nodeGoal);
            }
        }

        if (true) {                 // Animation
            system("clear");
            this->printMap();
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

    if (nodeCurrent == nodeGoal) {
        cout << "INFO : Goal Reached! With Iteration : " << iteration << endl;
        Node *nodePath = nodeGoal;
        while (nodePath->parents["Parent"] != nullptr) {
            this->plannedPath.push_back(nodePath);
            // Backtrack the parent
            nodePath = nodePath->parents["Parent"];
        }
    } else return false;   
}