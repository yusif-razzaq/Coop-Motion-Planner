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

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <cmath>

namespace ob = ompl::base;

ob::StateSpacePtr createBounded2ndOrderCarStateSpace(const std::vector<double>& dimensions) {
    ob::StateSpacePtr space = std::make_shared<ob::CompoundStateSpace>();
    space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
    space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
    space->as<ob::CompoundStateSpace>()->lock();
    
    // set the bounds for the RealVectorStateSpace 
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, dimensions[0]); //  x lower bound
    bounds.setHigh(0, dimensions[1]); // x upper bound
    bounds.setLow(1, dimensions[2]);  // y lower bound
    bounds.setHigh(1, dimensions[3]); // y upper bound
    bounds.setLow(2, -2);  // v lower bound
    bounds.setHigh(2, 2); // v upper bound
    bounds.setLow(3, -M_PI / 3);  // phi lower bound
    bounds.setHigh(3, M_PI / 3); // phi upper bound
    space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);

    return space;
}

ob::StateSpacePtr createBounded2ndOrderUAVStateSpace(const std::vector<double> dimensions) {
    ob::StateSpacePtr space = std::make_shared<ob::RealVectorStateSpace>(8);
    
    double maxSpeed = 3.0;
    double maxAngle = 0.2;
    // set the bounds for the RealVectorStateSpace 
    ob::RealVectorBounds bounds(8);
    bounds.setLow(0, dimensions[0]); //  x lower bound
    bounds.setHigh(0, dimensions[1]); // x upper bound
    bounds.setLow(1, dimensions[2]);  // y lower bound
    bounds.setHigh(1, dimensions[3]); // y upper bound
    bounds.setLow(2, -maxSpeed);        // x_dot lower bound (example value)
    bounds.setHigh(2, maxSpeed);        // x_dot upper bound (example value)
    bounds.setLow(3, -maxSpeed);        // y_dot lower bound (example value)
    bounds.setHigh(3, maxSpeed);        // y_dot upper bound (example value)
    bounds.setLow(4, -maxAngle);     // phi lower bound
    bounds.setHigh(4, maxAngle);    // phi upper bound
    bounds.setLow(5, -maxAngle);  // theta lower bound
    bounds.setHigh(5, maxAngle); // theta upper bound
    bounds.setLow(6, -maxAngle);       // phi_dot lower bound (example value)
    bounds.setHigh(6, maxAngle);       // phi_dot upper bound (example value)
    bounds.setLow(7, -maxAngle);       // theta_dot lower bound (example value)
    bounds.setHigh(7, maxAngle);       // theta_dot upper bound (example value)
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    return space;
}