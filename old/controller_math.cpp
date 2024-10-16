#include <iostream>
#include <cmath>

class RoboticArm {
private:
    float x, y; // Current coordinates of the robotic arm's end effector

public:
    RoboticArm(float initialX = 0.0, float initialY = 0.0) : x(initialX), y(initialY) {}

    // Function to update the position based on joystick input
    void updatePosition(float joystickLeft, float joystickRight) {
        // Scale joystick input to a maximum of 2 inches
        float xOffset = round(joystickLeft * 2.0 * 10) / 10.0;
        float yOffset = round(joystickRight * 2.0 * 10) / 10.0;

        // Update current position
        x += xOffset;
        y += yOffset;

        std::cout << "New Position -> X: " << x << " inches, Y: " << y << " inches\n";
    }

    // Function to get the current position
    void getPosition(float& currentX, float& currentY) {
        currentX = x;
        currentY = y;
    }
};

int main() {
    RoboticArm arm(0.0, 0.0); // Starting at origin

    // Simulate joystick inputs
    float leftJoystick = -1.0; // Up 100%
    float rightJoystick = 1.0; // Down 100%

    // Update the robotic arm position based on the joystick inputs
    arm.updatePosition(leftJoystick, rightJoystick);

    // Get and display the updated position
    float currentX, currentY;
    arm.getPosition(currentX, currentY);
    std::cout << "Updated Position -> X: " << currentX << ", Y: " << currentY << std::endl;

    return 0;
}
