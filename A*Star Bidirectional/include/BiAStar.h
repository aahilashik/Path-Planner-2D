#include <iostream>

#include <vector>
#include <list>

#include "../../PathPlanner/src/PathPlanner.cpp"

using namespace std;

class BiAStar : public PathPlanner2D {
    private:
        // Data Members
        list<Node*> openNodesList[2];

    public:
        /* -------- Constructors -------- */
        BiAStar() {
            this->setBidirectionalFlag(true);
        }

        template <size_t width, size_t height>
        BiAStar(int (&map2D)[width][height]) {
            this->updateMap(map2D);
            this->setBidirectionalFlag(true);
        }

        /* -------- Member Functions -------- */
        bool setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) override;
};
