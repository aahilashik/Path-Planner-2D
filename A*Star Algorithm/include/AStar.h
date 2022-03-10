#include <iostream>

#include <vector>
#include <list>

#include "../../PathPlanner/src/PathPlanner.cpp"

using namespace std;

class AStar : public PathPlanner2D {
    private:
        list<Node*> openNodesList;

    public:

        /* -------- Constructors -------- */
        AStar() {
            this->setBidirectionalFlag(false);
        }

        template <size_t width, size_t height>
        AStar(int (&map2D)[width][height]) {
            this->updateMap(map2D);
            this->setBidirectionalFlag(false);
        }

        /* -------- Member Functions -------- */
        bool setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) override;
};
