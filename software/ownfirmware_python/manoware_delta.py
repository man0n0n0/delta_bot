// Constants for robot geometry
f = fixed platform radius
e = end effector radius
rf = upper arm length
re = lower arm length
steps_per_rev = motor steps per revolution
gear_ratio = motor gear ratio

// Function to convert XYZ to motor steps
function cartesian_to_steps(x, y, z):
    // Calculate angles for each tower (120° apart)
    tower_angles = [0, 120, 240]
    motor_steps = []

    // Process each tower
    for i in 0 to 2:
        // Convert tower angle to radians
        theta = tower_angles[i] * PI / 180
        
        // Calculate tower position relative to center
        tower_x = f * cos(theta)
        tower_y = f * sin(theta)
        
        // Calculate end effector position relative to tower
        end_x = e * cos(theta)
        end_y = e * sin(theta)
        
        // Calculate distance from tower to end effector position
        dx = x + end_x - tower_x
        dy = y + end_y - tower_y
        dz = z
        
        // Calculate inverse kinematics
        dnorm = sqrt(dx² + dy² + dz²)
        E = (dx² + dy² + dz² + rf² - re² ) / (2 * rf)
        F = dz / dnorm
        
        // Calculate motor angle
        angle = asin(F) - acos(E / dnorm)
        
        // Convert angle to steps
        steps = angle * (steps_per_rev * gear_ratio) / (2 * PI)
        motor_steps.append(round(steps))
    
    return motor_steps

// Function to generate step/dir signals
function generate_step_signals(current_steps, target_steps):
    for i in 0 to 2:
        steps_to_move = target_steps[i] - current_steps[i]
        
        if steps_to_move > 0:
            direction[i] = CLOCKWISE
        else:
            direction[i] = COUNTER_CLOCKWISE
        
        steps_to_move = abs(steps_to_move)
        
        // Store movement data for each motor
        movement_data[i] = {
            "direction": direction[i],
            "steps": steps_to_move
        }
    
    return movement_data

// Main control loop
function main():
    while true:
        // Get target XYZ position
        target_position = get_next_position()
        
        // Calculate required steps for target position
        target_steps = cartesian_to_steps(
            target_position.x,
            target_position.y,
            target_position.z
        )
        
        // Generate step/dir signals
        movement = generate_step_signals(current_steps, target_steps)
        
        // Execute movement in parallel
        parallel_execute_movement(movement)
        
        // Update current position
        current_steps = target_steps
