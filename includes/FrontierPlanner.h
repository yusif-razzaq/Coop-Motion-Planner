
#include <ompl/control/PathControl.h>

void postFlightFrontier(const oc::PathControl& path, World *w) { 
    const int squareSize = 10;
    std::pair<std::size_t, std::size_t> gridSize = w->getGridSize();
    for (size_t i = 0; i < path.getStateCount(); ++i) {
        const ob::State* state = path.getState(i);
        const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();
        std::pair <std::size_t, std::size_t> centerCell = w->getCellFromPoint(realState->values[0], realState->values[1]);
        w->updateFrontier(centerCell);
    }
}
