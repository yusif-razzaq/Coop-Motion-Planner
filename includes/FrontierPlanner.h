
#include <ompl/control/PathControl.h>
#include "World.h"

double distanceXY(const std::vector<double>& state1, const std::vector<double>& state2) {
    return sqrt(pow(state1[0] - state2[0], 2) + pow(state1[1] - state2[1], 2));
}

std::pair<std::vector<double>, std::vector<double>> closestPointsPair(const std::vector<std::vector<double>>& set1, const std::vector<std::vector<double>>& set2) {

    double minDistance = std::numeric_limits<double>::max();
    std::pair<std::vector<double>, std::vector<double>> closestPair;

    for (const auto& point1 : set1) {
        for (const auto& point2 : set2) {
            double dist = distanceXY(point1, point2);
            if (dist < minDistance) {
                minDistance = dist;
                closestPair = {point1, point2};
            }
        }
    }

    return closestPair;
}

void postFlightFrontier(const oc::PathControl& path, World *w) { 
    const int squareSize = 10;
    std::pair<std::size_t, std::size_t> gridSize = w->getGridSize();
    for (size_t i = 0; i < path.getStateCount(); ++i) {
        const ob::State* state = path.getState(i);
        const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();
        std::pair <std::size_t, std::size_t> centerCell = w->getCellFromPoint(realState->values[0], realState->values[1]);
        w->updateFrontier(centerCell);
        // w->showGrid();
    }
}

class MyRRT2D {
    public:
        MyRRT2D(int n, double r, bool findGoal, const World *w) : n(n), r(r), findGoal(findGoal), w(w) {}
        bool plan(const std::vector<double>& init, const std::vector<double>& goal); 
        std::vector<double> getRandomPoint();
        std::pair<int, std::vector<double>> findNearest(const std::vector<double>& point, int& status);
        std::vector<std::vector<double>> getFrontierPoints() const {return frontierPoints;};
        void setLimits(const std::vector<std::pair<double, double>>& lims) {limits = lims;};
        void writeNodesToCSV(const std::string& filename);
        void writeParentsToCSV(const std::string& filename);
        void writeToCSV(const std::string& filename) {
            writeNodesToCSV("solutions/" + filename + "_nodes.csv");
            writeParentsToCSV("solutions/" + filename + "_connections.csv");
        };
    private:
        int n;
        double r;
        bool findGoal;
        std::map<uint32_t, std::vector<double>> points;
        std::vector<std::vector<double>> frontierPoints;
        std::map<uint32_t, uint32_t> parents;
        std::vector<std::pair<double, double>> limits;
        const World *w;
};

bool MyRRT2D::plan(const std::vector<double>& init, const std::vector<double>& goal) {
    std::cout << "Starting RRT from " << init[0] << ", " << init[1] << "\n";

    std::vector<double> qRand, nearest;
    points[0] = init;
    int ind = 1;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0, 1);    
    bool success = false;
    int status;
    while (points.size() < n) {
        double goalBias = dist(gen);
        if (goalBias > 0.95) qRand = goal;
        else qRand = getRandomPoint();
        // std::cout << "qRand at: "<< qRand[0] << ", " << qRand[1] << "\n";
        std::pair<int, std::vector<double>> nearest = findNearest(qRand, status);
        if (nearest.first != -1) {
            // std::cout << "Adding point: "<< nearest.second[0] << ", " << nearest.second[1] << "\n";
            points[ind] = nearest.second;
            parents[ind] = nearest.first;
            if (status == 1) frontierPoints.push_back(nearest.second);
            ind++;
            if (findGoal && distanceXY(nearest.second, goal) < 0.25) {
                std::cout << "Goal found in "<< ind << " steps\n";
                success = true;
                break;
            }
        }
    }
    ind--;
    int node = ind;
    std::vector<std::vector<double>> path;
    if (success) {
        while (node != 0) {
            path.insert(path.begin(), points[node]);
            node = parents[node];
        }
        path.insert(path.begin(), init);
        path.push_back(goal);
    } 
    return success;
}

std::pair<int, std::vector<double>> MyRRT2D::findNearest(const std::vector<double>& point, int& status) {
    std::vector<double> nearest = points[0];
    int ind = 0;
    for (int i = 1; i < points.size(); i++) {
        if (distanceXY(point, points[i]) < distanceXY(point, nearest)) {
            nearest = points[i];
            ind = i;
        }
    }
    // std::cout << "Closest node ind " << ind << std::endl;
    
    std::vector<double> direction = {point[0] - nearest[0], point[1] - nearest[1]};
    std::vector<double> step = {direction[0] / distanceXY(point, nearest) * r, direction[1] / distanceXY(point, nearest) * r};
    std::vector<double> endpoint = nearest;
    // std::cout << "Step vector: "<< step[0] << ", " << step[1] << "\n";

    int i = 0;
    while (distanceXY(point, nearest) > distanceXY(endpoint, nearest)) {
        endpoint = {endpoint[0] + step[0], endpoint[1] + step[1]};
        status = w->gridStatus(endpoint[0], endpoint[1]);
        // std::cout << "Endpoint: "<< endpoint[0] << ", " << endpoint[1] << "\n";
        // std::cout << "Status: " << status << std::endl;

        if (status != 0) { 
            if (i == 0) ind = -1;
            break;
        }
        i++;
    }
    endpoint = {endpoint[0] - step[0], endpoint[1] - step[1]};
    return {ind, endpoint};
}

std::vector<double> MyRRT2D::getRandomPoint() {
    int dim = 2;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::pair<double, double> limit;
    std::vector<double> randomPoint;
    for (int i = 0; i < dim; i++) {
        limit = limits[i];
        std::uniform_real_distribution<double> dist(limit.first, limit.second);
        randomPoint.push_back(dist(gen));
    }
    return randomPoint;
}

void MyRRT2D::writeNodesToCSV(const std::string& filename) {
    std::ofstream file(filename);
    for (const auto& node : points) {
        file << node.first << ",";  // Write node index

        // Write node coordinates
        const std::vector<double>& coordinates = node.second;
        for (size_t i = 0; i < coordinates.size(); ++i) {
            file << coordinates[i];
            if (i != coordinates.size() - 1) file << ",";
        }
        file << std::endl;
    }

    file.close();
}

void MyRRT2D::writeParentsToCSV(const std::string& filename) {
    std::ofstream file(filename);
    for (const auto& parent : parents) {
        file << parent.first << "," << parent.second << std::endl;
    }
    file.close();
}

std::pair<std::vector<double>, std::vector<double>> findBestFrontierPoints(World *w, const Agent *a) { 
    auto limits = w->getWorldDimensions();
    MyRRT2D startRRT(100, 0.25, false, w);
    startRRT.setLimits({{limits[0], limits[1]}, {limits[2], limits[3]}});
    startRRT.plan(a->getStartLocation(), a->getGoalLocation());
    startRRT.writeToCSV("start");
    MyRRT2D goalRRT(100, 0.25, false, w);
    goalRRT.setLimits({{limits[0], limits[1]}, {limits[2], limits[3]}});
    goalRRT.plan(a->getGoalLocation(), a->getStartLocation());
    goalRRT.writeToCSV("goal");
    return closestPointsPair(startRRT.getFrontierPoints(), goalRRT.getFrontierPoints());
}