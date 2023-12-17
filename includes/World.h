/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */
#pragma once
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <ompl/tools/config/SelfConfig.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

// rectangular Obstacle defined by two points (xMin, yMin) and (xMax, yMax)
struct Obstacle {
    const double xMin_;
    const double yMin_;
    const double xMax_;
    const double yMax_;
    const polygon poly_;

    void printPoints() const
    {
        auto exterior_points = boost::geometry::exterior_ring(poly_);
        for (int i=0; i<exterior_points.size(); i++)
        {
            std::cout << boost::geometry::get<0>(exterior_points[i]) << " " << boost::geometry::get<1>(exterior_points[i]) <<std::endl;
        }
    }
    polygon::ring_type getPoints() const
    {
        return boost::geometry::exterior_ring(poly_);
    }
};

enum CellState {
    KNOWN,
    UNKNOWN
};

struct GridCell {
    CellState state;
    // Other information if needed, e.g., cost, probability, etc.
};

// An agent has a name, shape, dynamics, start and goal regions
// Created as class to keep important variables safe
class Agent {
    public:
        Agent(std::string name, std::string dyn, std::vector<double> shape, std::vector<double> s, std::vector<double> g) {
            name_ = name;
            dynamics_ = dyn;
            shape_ = shape;
            start_ = s;
            goal_ = g;
        }
        std::string getName() const {return name_;};
        std::string getDynamics() const {return dynamics_;};
        std::vector<double> getShape() const {return shape_;};
        std::vector<double> getStartLocation() const {return start_;};
        std::vector<double> getGoalLocation() const {return goal_;};
        void setStartLocation(const std::vector<double>& start) {start_ = start;};
        void setGoalLocation(const std::vector<double>& goal) {goal_ = goal;};
    private:
        std::string name_;
        std::string dynamics_;
        std::vector<double> shape_;
        std::vector<double> start_;
        std::vector<double> goal_;
};



// world class holds all relevent data in the world that is used by OMPL
class World {
    public:
        World() : occupancyGrid(80, std::vector<GridCell>(80, {UNKNOWN})){  // 0.2 m cellWidth
            rows = occupancyGrid.size(); // Number of rows
            cols = (rows > 0) ? occupancyGrid[0].size() : 0; // Number of columns
        }
        // methods for dimensions
        void setWorldDimensions(const std::vector<double>& dim) {dimensions = dim;};
        std::vector<double> getWorldDimensions() const {return dimensions;};
        void printWorldDimensions(){OMPL_INFORM("Space Dimensions: [%0.2f, %0.2f, %0.2f, %0.2f]", dimensions[0], dimensions[1], dimensions[2], dimensions[3] );}
        // methods for obstacles
        void addObstacle(Obstacle obs){Obstacles_.push_back(obs);};
        std::vector<Obstacle> getObstacles() const {return Obstacles_;};
        // methods for agents
        void addAgent(Agent *a){Agents_.push_back(a);};
        std::vector<Agent*> getAgents() const {return Agents_;};
        // printing methods for usability
        void printObstacles() {
            OMPL_INFORM("%d Obstacle(s) (xMin, yMin, xMax, yMax): ", Obstacles_.size());
            for (Obstacle o: Obstacles_) {
                OMPL_INFORM("   - Obstacle: [%0.2f, %0.2f, %0.2f, %0.2f]", o.xMin_, o.yMin_, o.xMax_, o.yMax_);
            }
        }
        void printAgents() {
            OMPL_INFORM("%d Agents: ", Agents_.size());
            for (Agent *a: Agents_) {
                OMPL_INFORM("   - Name: %s", a->getName().c_str());
                OMPL_INFORM("     Dynamics: %s", a->getDynamics().c_str());
                OMPL_INFORM("     Start: [%0.2f, %0.2f]", a->getStartLocation()[0], a->getStartLocation()[1]);
                OMPL_INFORM("     Goal: [%0.2f, %0.2f]", a->getGoalLocation()[0], a->getGoalLocation()[1]);
            }
        }
        void printWorld() {
            printWorldDimensions();
            printObstacles();
            printAgents();
        }

        std::pair<std::size_t, std::size_t> getGridSize() const {return {rows, cols};};

        std::pair<std::size_t, std::size_t> getCellFromPoint(double x, double y) const {
            double cellWidth0 = (dimensions[1] - dimensions[0]) / rows;
            double cellWidth1 = (dimensions[3] - dimensions[2]) / cols;
            int i = std::floor((x - dimensions[0]) / cellWidth0);
            int j = std::floor((y - dimensions[2]) / cellWidth1); 
            return std::pair<std::size_t, std::size_t>(i, j);
        };

        void updateFrontier(const std::pair<std::size_t, std::size_t>& centerCell) {
            int squareSize = 15; // 5 m
            int startX = centerCell.first - squareSize / 2;
            if (startX < 0) startX = 0;
            int startY = centerCell.second - squareSize / 2;
            if (startY < 0) startY = 0;
            int endX = centerCell.first + squareSize / 2;
            if (endX >= rows) endX = rows - 1;
            int endY = centerCell.second + squareSize / 2;
            if (endY >= cols) endY = cols - 1;
            for (int i = startX; i <= endX; ++i) {
                for (int j = startY; j <= endY; ++j) {
                    occupancyGrid[i][j].state = KNOWN;
                }
            }
        }

        bool isGridKnown(double x, double y) const {
            if (x < dimensions[0] || x > dimensions[1] || y < dimensions[2] || y > dimensions[3]) return false;
            std::pair<std::size_t, std::size_t> cell = getCellFromPoint(x, y);
            // std::cout << "checking point " << x << ", " << y << "\n";
            // std::cout << "checking cell " << cell.first << ", " << cell.second << "\n";
            if (occupancyGrid[cell.first][cell.second].state == KNOWN) return true;
            else return false;
        }

        void showGrid() {
            cv::Mat occupancyMap(rows, cols, CV_8UC3);
            // Set colors for different cell states
            cv::Scalar colorKnown(108, 108, 0);       // White for free cells
            cv::Scalar colorUnknown(63, 63, 171);   // Gray for unknown cells
            cv::Vec3b colorKnownVec = cv::Vec3b(colorKnown[0], colorKnown[1], colorKnown[2]);
            cv::Vec3b colorUnknownVec = cv::Vec3b(colorUnknown[0], colorUnknown[1], colorUnknown[2]);
            // Fill the occupancy map with corresponding colors based on cell states
            for (int i = 0; i < occupancyMap.rows; ++i) {
                for (int j = 0; j < occupancyMap.cols; ++j) {
                    if (occupancyGrid[i][j].state == KNOWN) {
                        occupancyMap.at<cv::Vec3b>(j, i) = colorKnownVec;
                    } else {
                        occupancyMap.at<cv::Vec3b>(j, i) = colorUnknownVec;
                    }
                }
            }      
            cv::imshow("Occupancy Grid", occupancyMap);
            cv::waitKey(0);  // Wait for a key press to close the window 
        }

        int gridStatus(double x, double y) const {
            if (isPointInsidePolygons(x, y)) return 2;
            if (isGridKnown(x, y)) return 0;
            return 1;
        }

        bool isPointInsidePolygons(double x, double y) const {
            for (Obstacle obstacle: Obstacles_) {
                polygon::ring_type vertices = obstacle.getPoints();
                int numVertices = vertices.size();
                bool inside = false;
                for (int i = 0, j = numVertices - 1; i < numVertices; j = i++) {
                    double vx1 = boost::geometry::get<0>(vertices[i]);
                    double vy1 = boost::geometry::get<1>(vertices[i]);
                    double vx2 = boost::geometry::get<0>(vertices[j]);
                    double vy2 = boost::geometry::get<1>(vertices[j]);
                    if ((vy1 > y) != (vy2 > y) && x < (vx2 - vx1) * (y - vy1) / (vy2 - vy1) + vx1) {
                        inside = !inside;
                    }
                }
                if (inside) return inside;
            }
            return false;
        }
 
    private:
        std::vector<double> dimensions;
        std::vector<Obstacle> Obstacles_;
        std::vector<Agent*> Agents_;
        std::vector<std::vector<GridCell>> occupancyGrid;
        std::size_t rows;
        std::size_t cols; 
};

// function that parses YAML file to world object
World* yaml2world(std::string file) {
    YAML::Node config;
    World *w = new World();
    try {
        OMPL_INFORM("Path to Problem File: %s", file.c_str());
        config = YAML::LoadFile(file);
        std::cout << "" << std::endl;
        OMPL_INFORM("File loaded successfully. Parsing...");
    }
    catch (const std::exception& e) {
        OMPL_ERROR("Invalid file path. Aborting prematurely to avoid critical error.");
        exit(1);
    }
    try {    
        // grab dimensions from problem definition
        const auto& dims = config["Map"]["Dimensions"];
        const double x_min = dims[0].as<double>();
        const double x_max = dims[1].as<double>();
        const double y_min = dims[2].as<double>();
        const double y_max = dims[3].as<double>();
        w->setWorldDimensions({x_min, x_max, y_min, y_max});
        
        // set Obstacles
        const auto& obs = config["Map"]["Obstacles"];
        for (int i=0; i < obs.size(); i++) {
            std::string name = "obstacle" + std::to_string(i);
            const double minX = obs[name][0].as<double>();
            const double minY = obs[name][1].as<double>();
            const double maxX = obs[name][2].as<double>();
            const double maxY = obs[name][3].as<double>();            
            // TOP RIGHT VERTEX:
            std::string top_right = std::to_string(maxX) + " " + std::to_string(maxY);
            // TOP LEFT VERTEX:
            std::string top_left = std::to_string(minX) + " " + std::to_string(maxY);
            // BOTTOM LEFT VERTEX:
            std::string bottom_left = std::to_string(minX) + " " + std::to_string(minY);
            // BOTTOM RIGHT VERTEX:
            std::string bottom_right = std::to_string(maxX) + " " + std::to_string(minY);

            // convert to string for easy initializataion
            std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
            polygon poly;
            boost::geometry::read_wkt(points,poly);
            Obstacle o = {minX, minY, maxX, maxY, poly};
            w->addObstacle(o);
        }

        // Define the agents
        const auto& agents = config["Agents"];
        for (int i=0; i < agents.size(); i++) {
            std::string name = "agent" + std::to_string(i);
            const std::vector<double> shape{agents[name]["Shape"][0].as<double>(), agents[name]["Shape"][1].as<double>()};
            const std::vector<double> start{agents[name]["Start"][0].as<double>(), agents[name]["Start"][1].as<double>()};
            const std::vector<double> goal{agents[name]["Goal"][0].as<double>(), agents[name]["Goal"][1].as<double>()};
            Agent *a = new Agent(name, agents[name]["Model"].as<std::string>(), shape, start, goal);
            w->addAgent(a);
        }
        OMPL_INFORM("Parsing Complete.");
        std::cout << "" << std::endl;
        w->printWorld();
    }
    catch (const std::exception& e) {
        OMPL_ERROR("Error During Parsing. Aborting prematurely to avoid critical error.");
        exit(1);
    }
    return w;
}
