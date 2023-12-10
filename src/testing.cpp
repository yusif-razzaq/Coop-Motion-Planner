#include <iostream>
#include <cmath>

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