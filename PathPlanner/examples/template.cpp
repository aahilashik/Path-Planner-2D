#include <iostream>

#include <vector>
#include <list>

#include <PathPlanner.cpp>

using namespace std;

const int mapWidth      = 32;
const int mapHeight     = 32;

// 2D Map Array 
int map2D[mapWidth][mapHeight] =    {   {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                        {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}   };

class newPPAlgo : public PathPlanner2D {
    public:
        /* -------- Constructors -------- */
        newPPAlgo() {}

        template <size_t width, size_t height>
        newPPAlgo(int (&map2D)[width][height]) {
            this->updateMap(map2D);
        }

        /* -------- Member Functions -------- */
        void setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) override;
};

void newPPAlgo::setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) {
    // Initialize the Nodes here
    for (int x = 0; x < this->mapWidth; x++)
        for (int y = 0; y < this->mapHeight; y++) {

            // Use costMap member variable of map<string, double> type
            // to store cost parameters like path cost, state cost, etc.
            // Syntax: 
            //  nodes[<x> * this->mapWidth + <y>].costMap["<cost_name>"] = <value>;
            // Examples

            nodes[x * this->mapWidth + y].costMap["PathCost"] = INFINITY;
            nodes[x * this->mapWidth + y].costMap["TotalCost"] = INFINITY;

            nodes[x * this->mapWidth + y].parent = nullptr;	// No parents
        }

    // Setup starting conditions
    Node *nodeCurrent = nodeStart;
    nodeStart->costMap["PathCost"] = 0.0f;
    nodeStart->costMap["TotalCost"] = heuristic(nodeStart, nodeGoal);


    list<Node*> openNodesList;
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
            if (!nodeNeighbour->nodeState["isExplored"] && !nodeNeighbour->nodeState["isObstacle"])
                openNodesList.push_back(nodeNeighbour);

            float fPossiblyLowerGoal = nodeCurrent->costMap["PathCost"] + distance(nodeCurrent, nodeNeighbour);

            if (fPossiblyLowerGoal < nodeNeighbour->costMap["PathCost"]) {
                nodeNeighbour->parent = nodeCurrent;
                nodeNeighbour->costMap["PathCost"] = fPossiblyLowerGoal;
                nodeNeighbour->costMap["TotalCost"] = nodeNeighbour->costMap["PathCost"] + heuristic(nodeNeighbour, nodeGoal);
            }
        }	
    }

    if (nodeCurrent == nodeGoal) {
        cout << "INFO : Goal Reached! With Iteration : " << iteration << endl;
        Node *nodePath = nodeGoal;
        while (nodePath->parent != nullptr) {
            this->plannedPath.push_back(nodePath);
            // Backtrack the parent
            nodePath = nodePath->parent;
        }
    }   
}

int main() {
        newPPAlgo aStar = newPPAlgo(map2D);
        aStar.setStartNode(0, 0);
        aStar.setGoalNode(15, 15);
        aStar.startPlan();

        vector<PathPlanner2D::Node*> path;
        aStar.getPlannedPath(path);
        for (auto p: path) {
            map2D[p->x][p->y] = -1;
            cout << p->x << " - " << p->y << endl;
        }
    
        aStar.printMap(path);
}

// g++ -I /home/ubuntussd/cpp_ws/Robotics/01_COBOT_ARM/002_Path_Planner/PathPlanner/src/  01_a_star.cpp -o code && ./code