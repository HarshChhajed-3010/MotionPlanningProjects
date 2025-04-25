#include "RG-RRT.h"

// Include other necessary headers
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Planner.h>
#include <memory>

// ...existing code...

// Create the planner using RGRRT
auto planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());

// ...existing code...