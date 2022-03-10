/*************************************************************************
	> File Name: PathPlanner.h
	> Author: Muhammath Ashik Farhan S
	> Mail: aahilashik@gmail.com
	> Created on: Jan 2022
	> Description: Base Header Code 2D Grid Based Path Planner
************************************************************************/

#include <iostream>

#include <vector>
#include <map>

using namespace std;

class PathPlanner2D{
    public:
        // Node Structure
        struct Node {
            // Coordinates/Pose of the Node
            int     x, y;

            // To store Cost and Pose Related Parameter
            map<string, double> costMap;
            map<string, bool>   nodeState;

            // Vector/List of Neighbours
            vector<Node*>   nodeNeighbours;

            // Parent Node of each node
            map<string, Node*> parents;
        };

        // Planned Path List  
        list<Node*> plannedPath;

        // Map Dimensions
        int mapWidth    = 0;
        int mapHeight   = 0;

        // Meeting Node if Bidirectional
        Node *nodeMiddle = nullptr;

    private:
        // Nodes Array
        Node *nodes = nullptr;
        
        // Start & Goal Nodes
        Node *nodeStart = nullptr;
        Node *nodeGoal = nullptr;

        // Whether Bidirectional or Unidirectional
        bool isBidirectional = false;
        
    public:
        // Constructors

            PathPlanner2D();

            // Constructor with 2D Map as input
            template <size_t width, size_t height>
            PathPlanner2D(int (&map2D)[width][height]);

        // Member Functions

            void setBidirectionalFlag(bool flag);
        
            // Function with 2D Map as input
            template <size_t width, size_t height>
            void updateMap(int (&map2D)[width][height]);

            void setStartNode(int x, int y);
            void setGoalNode(int x, int y);

            bool startPlan();
            
            void getPlannedPath(vector<Node*>& path);
            virtual bool setPlannedPath(Node *nodes, Node *Start, Node *Goal); 

            void resetNodes();
            void printMap(vector<Node*> path);
};
