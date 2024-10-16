class RoboticArm:
    def __init__(self, initial_x=0.0, initial_y=0.0):
        self.x = initial_x
        self.y = initial_y

    def update_position(self, joystick_left, joystick_right):
        # Scale joystick input to a maximum of 2 inches
        x_offset = round(joystick_left * 2.0, 1)
        y_offset = round(joystick_right * 2.0, 1)

        # Update current position
        self.x += x_offset
        self.y += y_offset

        print(f"New Position -> X: {self.x} inches, Y: {self.y} inches")

    def get_position(self):
        return self.x, self.y

if __name__ == "__main__":
    arm = RoboticArm()  # Start at the origin

    # Simulate joystick inputs
    left_joystick = -0.5  # Up 100%
    right_joystick = 0.5  # Down 100%

    # Update the robotic arm position based on the joystick inputs
    arm.update_position(left_joystick, right_joystick)

    # Get and display the updated position
    current_x, current_y = arm.get_position()
    print(f"Updated Position -> X: {current_x}, Y: {current_y}")
