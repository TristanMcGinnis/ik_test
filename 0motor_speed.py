def calculate_duty_cycle(degrees_to_travel, time_to_travel, axis):
    # Constants based on given data
    k_values = {
        'Axis1': 41.03,  # % per RPM
        'Axis2': 18.88,  # % per RPM
        'Axis3': 13.08   # % per RPM
    }
    
    # Calculate the desired RPM
    desired_rpm = (degrees_to_travel * 60) / (time_to_travel * 360)
    
    # Calculate the duty cycle based on the axis
    duty_cycle = k_values[axis] * desired_rpm
    return duty_cycle

# Example usage
degrees_to_travel = 90
time_to_travel = 15  # seconds
axis = 'Axis3'
duty_cycle = calculate_duty_cycle(degrees_to_travel, time_to_travel, axis)
print(f"Required Duty Cycle for {axis} to travel {degrees_to_travel} degrees in {time_to_travel} seconds: {duty_cycle:.2f}%")
