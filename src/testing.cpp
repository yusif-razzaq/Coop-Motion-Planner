#include <iostream>
#include <cmath>

double wrapToPi(double angle) {
    angle = std::fmod(angle + M_PI, 2 * M_PI); // Adjust to the range [0, 2π]
    if (angle < 0)
        angle += 2 * M_PI; // Ensure angle is positive

    if (angle > M_PI)
        angle -= 2 * M_PI; // Bring back to the range [-π, π]

    return angle;
}

double wrapToPi(double angle) {
    angle = std::fmod(angle, 2.0 * M_PI);
    if (angle < -M_PI)
        angle += 2.0 * M_PI;
    else if (angle > M_PI)
        angle -= 2.0 * M_PI;
    
    return angle;
}

int main() {
    double angle = -6.1; // Angle in radians
    double wrapped_angle = wrapToPi(angle);

    std::cout << "Wrapped angle: " << wrapped_angle << std::endl;

    return 0;
}