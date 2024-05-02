#include <Arduino.h>
#include <math.h>

class ik_arm {
  public:
    float l1, l2, l3; // Lengths of the arm segments
    float theta1, theta2, theta3; // Joint angles in radians

    // Constructor
    ik_arm(float length1, float length2, float length3) {
      l1 = length1;
      l2 = length2;
      l3 = length3;
      theta1 = 0;
      theta2 = 0;
      theta3 = 0;
    }

    // Calculate joint angles using inverse kinematics for reaching (x, y)
    void calculate(float x, float y) {
      float targetDist = sqrt(x * x + y * y); // Distance from base to point

      // First check if the point is reachable
      if (targetDist > l1 + l2 + l3) {
        Serial.println("Position out of reach");
        return; // Position out of reach
      }

      // Approximate approach using an intermediate target (the reach of the first two segments)
      float effectiveLength = sqrt(x*x + y*y - l3*l3);
      float alpha = atan2(y, x);
      float beta = acos((l1*l1 + effectiveLength*effectiveLength - l2*l2) / (2 * l1 * effectiveLength));
      theta1 = alpha - beta;

      float gamma = acos((l1*l1 + l2*l2 - effectiveLength*effectiveLength) / (2 * l1 * l2));
      theta2 = PI - gamma; // elbow down

      // Adjust theta3 based on the current orientation of the arm
      theta3 = atan2(y, x) - theta1 - theta2;

      // Store the angles in degrees for easier usage
      theta1 = theta1 * 180 / PI;
      theta2 = theta2 * 180 / PI;
      theta3 = theta3 * 180 / PI;
    }

    // Print the current joint angles
    void printAngles() {
      Serial.print("Theta1: ");
      Serial.print(theta1);
      Serial.print(" degrees, Theta2: ");
      Serial.print(theta2);
      Serial.print(" degrees, Theta3: ");
      Serial.print(theta3);
      Serial.println(" degrees");
    }
};

void setup() {
  Serial.begin(9600);
  ik_arm myArm(10.0, 8.0, 6.0); // Create an instance with segment lengths 10, 8, and 6
  myArm.calculate(12.0, 9.0); // Calculate angles to reach the point (12, 9)
  myArm.printAngles(); // Print calculated angles
}

void loop() {
  // Here the loop is left empty as no continuous action is required
}
