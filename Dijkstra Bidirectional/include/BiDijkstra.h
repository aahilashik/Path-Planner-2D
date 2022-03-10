#include <iostream>

#include <vector>
#include <list>

#include "../../PathPlanner/src/PathPlanner.cpp"

using namespace std;

class BiDijkstra : public PathPlanner2D {
    private:
        // Data Members
        list<Node*> openNodesList[2];

    public:
        /* -------- Constructors -------- */
        BiDijkstra() {
            this->setBidirectionalFlag(true);
        }

        template <size_t width, size_t height>
        BiDijkstra(int (&map2D)[width][height]) {
            this->setBidirectionalFlag(true);
            this->updateMap(map2D);
        }

        /* -------- Member Functions -------- */
        bool setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) override;
};
