#include "RG-RRT.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Planner.h>
#include <memory>

// If you're creating the planner, make sure to use:
auto planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());

int main() {
    // Your main function implementation here
    return 0;
}