#include <iostream>

#include <vector>
#include <list>

#include "../../PathPlanner/src/PathPlanner.cpp"

using namespace std;

class yourPPAlgo : public PathPlanner2D {
    public:

        /* -------- Constructors -------- */
        yourPPAlgo() {
            // Set flag as true if your path planning algorithm is bidirectional
            bool flag = false;
            this->setBidirectionalFlag(flag);
        }

        template <size_t width, size_t height>
        yourPPAlgo(int (&map2D)[width][height]) {
            this->updateMap(map2D);
        
            // Set flag as true if your path planning algorithm is bidirectional
            bool flag = false;
            this->setBidirectionalFlag(flag);
        }

        /* -------- Member Functions -------- */
        bool setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) override;
};
