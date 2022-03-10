#include <iostream>

#include <vector>
#include <list>

#include "../../PathPlanner/src/PathPlanner.cpp"

using namespace std;

class GreedyBestFirstSearch : public PathPlanner2D {
    private:
        // Data Members
        list<Node*> openNodesList;

    public:
        /* -------- Constructors -------- */
        GreedyBestFirstSearch() {
            this->setBidirectionalFlag(false);
        }

        template <size_t width, size_t height>
        GreedyBestFirstSearch(int (&map2D)[width][height]) {
            this->updateMap(map2D);
            this->setBidirectionalFlag(false);
        }

        /* -------- Member Functions -------- */
        bool setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) override;
};
