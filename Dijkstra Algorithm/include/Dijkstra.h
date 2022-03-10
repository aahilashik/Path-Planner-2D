#include <iostream>

#include <vector>
#include <list>

#include "../../PathPlanner/src/PathPlanner.cpp"

using namespace std;

class Dijkstra : public PathPlanner2D {
    private:
        // Data Members
        list<Node*> openNodesList;

    public:
        /* -------- Constructors -------- */
        Dijkstra() {
            this->setBidirectionalFlag(false);
        }

        template <size_t width, size_t height>
        Dijkstra(int (&map2D)[width][height]) {
            this->updateMap(map2D);
            this->setBidirectionalFlag(false);
        }

        /* -------- Member Functions -------- */
        bool setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) override;
};
