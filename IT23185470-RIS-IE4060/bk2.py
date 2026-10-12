import pygame
import math

# ---------------------------------------------------------
# --01-- Initialize Pygame
# ---------------------------------------------------------

pygame.init()

#window size
WIDTH = 900
HEIGHT = 600

#create simulation window
screen = pygame.display.set_mode((WIDTH, HEIGHT))

#window title
pygame.display.set_caption(
    "Intelligent Differential Drive Robot Simulation"
    )

#--------------------------------------------
#clock controls the simulation frame rate 
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 20)

#Robot State
x = 150.0
y = 450.0
theta = math.radians(-20)

#distance between left and right wheels
wheel_base = 50

#traget position
traget_x = 750
traget_y = 150

#---------------------------------------------------------
# P Controller Parameters
#---------------------------------------------------------

#Proportional gain for distance control
Kp_distance = 0.8

#Derivative gain for PD distance control
Kd_distance = 0.25

#Proportional gain for heading control
Kp_heading = 3.0

#maximum robot velocities
max_linear_velocity = 150.0
max_angular_velocity = 2.5

#wheel radius
wheel_radius = 15.0

#robot stops when it is close enough to the target
stop_tolerance = 8.0

#simulation time step (seconds)
dt = 1/60.0

# Simulation measurements
simulation_time = 0.0
current_linear_velocity = 0.0
final_time = None

#previous distance error used by the derivative term
previous_distance_error = None

#current controller
controller_mode = "p"

# Store robot path
trajectory = [(int(x), int(y))]

#colours
BLUE = (60, 110, 220)
RED = (220, 60, 60)
BLACK = (30, 30, 30)
DARK_GRAY = (70, 70, 70)
GREEN = (60, 170, 90)
ORANGE = (240, 150, 40)

#---------------------------------------------------
# Draw the robot body
def draw_robot():
    robot_Position = (
        int(x),
        int(y)
    )

    #Robot body
    pygame.draw.circle(
        screen,
        BLUE,
        robot_Position,
        22
    )

    #Robot border
    pygame.draw.circle(
        screen,
        BLACK,
        robot_Position,
        22,
        2
    )

    #Heading direction
    heading_length = 45

    heading_x = (x + heading_length * math.cos(theta))
    heading_y = (y + heading_length * math.sin(theta))

    pygame.draw.line(
        screen,
        BLACK,
        robot_Position,
        (int(heading_x), int(heading_y)),
        4
    )

    #Left and right wheels positions
    perpendicular_angle = (theta + math.pi / 2)
    wheel_offset = wheel_base / 2

    left_wheel_x = (x + wheel_offset * math.cos(perpendicular_angle))
    left_wheel_y = (y + wheel_offset * math.sin(perpendicular_angle))

    right_wheel_x = (x - wheel_offset * math.cos(perpendicular_angle))
    right_wheel_y = (y - wheel_offset * math.sin(perpendicular_angle))

    pygame.draw.circle(
        screen,
        DARK_GRAY,
        (int(left_wheel_x), int(left_wheel_y)),
        7
    )

    pygame.draw.circle(
        screen,
        DARK_GRAY,
        (int(right_wheel_x), int(right_wheel_y)),
        7
    )

#-----------------------------------------------
#Draw the target position
def draw_target():
    pygame.draw.circle(
        screen,
        RED,
        (traget_x, traget_y),
        15
    )

    pygame.draw.circle(
        screen,
        BLACK,
        (traget_x, traget_y),
        15,
        2
    )

#-----------------------------------------------
#Draw the robot trajectory
def draw_trajectory():
    if len(trajectory) > 1:
        pygame.draw.lines(
            screen,
            ORANGE,
            False, #last point is not connected to the first point
            trajectory,
            3
        )

#-----------------------------------------------
#Normalize angle to be between -pi and pi
def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )

#-----------------------------------------------
#calculate error
def calculate_errors():
    #position error in X direction
    error_x = traget_x - x

    #position error in Y direction
    error_y = traget_y - y

    #distance error
    distance_error = math.sqrt(
        error_x ** 2 + error_y ** 2
    )

    #Desired heading angle
    desired_heading = math.atan2(
        error_y,
        error_x
    )

    #heading error
    heading_error = (
        desired_heading - theta
    )

    #keep angle between -pi and pi
    heading_error = normalize_angle(heading_error)

    return (
        error_x,
        error_y,
        distance_error,
        desired_heading,
        heading_error
    )

#-----------------------------------------------
#actual p Controller + Differential Drive Robot motion
def p_control_update():
    global x, y, theta
    global simulation_time
    global current_linear_velocity
    global final_time

    #get current errors
    (
        error_x,
        error_y,
        distance_error,
        desired_heading,
        heading_error
    ) = calculate_errors()

    ##Stopping condition
    if distance_error <= stop_tolerance:
        current_linear_velocity = 0.0
        if final_time is None:
            final_time = simulation_time
        return

    #--------------------------------------
    ##p control for linear velocity
    ##v= Kp_distance * distance_error
    linear_velocity = (Kp_distance * distance_error)

    #limit maximum linear velocity
    linear_velocity = min(linear_velocity, max_linear_velocity)

    #--------------------------------------
    ##p control for angular velocity
    ##omega= Kp_heading * heading_error
    angular_velocity = (Kp_heading * heading_error)

    #limit maximum angular velocity
    angular_velocity = max(
        -max_angular_velocity,
        min(angular_velocity, max_angular_velocity)
    )

    #--------------------------------------
    ##convert liner and angular velocity
    ##into left and right wheel angular velocities
    right_wheel_velocity = (2 * linear_velocity + angular_velocity * wheel_base) / (2 * wheel_radius)
    left_wheel_velocity = (2 * linear_velocity - angular_velocity * wheel_base) / (2 * wheel_radius)

    #--------------------------------------
    ##Differential drive Robot kinematics
    actual_linear_velocity = ( wheel_radius / 2) * (right_wheel_velocity + left_wheel_velocity)
    actual_angular_velocity = ( wheel_radius / wheel_base) * (right_wheel_velocity - left_wheel_velocity)

    #--------------------------------------
    ##position and orientation update
    x = x + (actual_linear_velocity * math.cos(theta) * dt)
    y = y + (actual_linear_velocity * math.sin(theta) * dt)
    theta = theta + (actual_angular_velocity * dt)

    #Normalize orientation
    theta = normalize_angle(theta)
    current_linear_velocity = actual_linear_velocity
    simulation_time +=dt

    #Append current position to trajectory
    trajectory.append((int(x), int(y)))

#-----------------------------------------------
# PD Controller + Differential Drive Robot Motion
def pd_control_update():
    global x,y, theta
    global previous_distance_error
    global simulation_time
    global current_linear_velocity
    global final_time

    #get current errors
    (
        error_x,
        error_y,
        distance_error,
        desired_heading,
        heading_error
    ) = calculate_errors()

    ##Stopping condition
    if distance_error <= stop_tolerance:
        current_linear_velocity = 0.0
        if final_time is None:
            final_time = simulation_time
        return

    ##--------------------------------------
    ##Derivative of distance error
    ##de/dt = (current error - previous error) / dt
    if previous_distance_error is None:
        distance_derivative = 0.0
    else:
        distance_derivative = (distance_error - previous_distance_error) / dt

    ##--------------------------------------
    ##PD control for linear velocity
    ##v = kp*ed + kd*(ded/dt)
    linear_velocity = (
        Kp_distance * distance_error + Kd_distance * distance_derivative
    )

    #do not allow reverse motion
    linear_velocity = max(0.0, linear_velocity)

    #Limit maximum linear velocity
    linear_velocity = min(linear_velocity, max_linear_velocity)

    ##---------------------------------------
    ##P Control for Heading
    ##omega = Kp_heading * heading_error
    angular_velocity = (Kp_heading * heading_error)
    # Limit angular velocity
    angular_velocity = max(-max_angular_velocity, min(angular_velocity, max_angular_velocity))

    ##---------------------------------------
    ##Convert Linear and Angular Velocity
    ##to Left and Right Wheel Angular Velocities 
    right_wheel_velocity = ( 2 * linear_velocity + angular_velocity * wheel_base) / ( 2 * wheel_radius )
    left_wheel_velocity = ( 2 * linear_velocity - angular_velocity * wheel_base ) / ( 2 * wheel_radius)

    ##----------------------------------------
    # Differential Drive Robot Kinematics
    actual_linear_velocity = ( wheel_radius / 2) * ( right_wheel_velocity + left_wheel_velocity)
    actual_angular_velocity = ( wheel_radius / wheel_base) * ( right_wheel_velocity - left_wheel_velocity)

    ##----------------------------------------
    ##Position and Orientation Update 
    x = x + ( actual_linear_velocity * math.cos(theta) * dt )
    y = y + ( actual_linear_velocity * math.sin(theta) * dt )

    theta = theta + ( actual_angular_velocity * dt)

    ##Normalize orientation
    theta = normalize_angle(theta)
    current_linear_velocity = actual_angular_velocity
    simulation_time += dt

    ##----------------------------------------
    ##save current error for next frame
    previous_distance_error = distance_error

    ##----------------------------------------
    ##store current position for trajectory
    trajectory.append((int(x),int(y)))

#-----------------------------------------------
#draw final stopping position
def draw_final_position():
    (
        error_x,
        error_y,
        distance_error,
        desired_heading,
        heading_error
    ) = calculate_errors()
    if distance_error <= stop_tolerance:
        pygame.draw.circle(
            screen,
            GREEN,
            (int(x), int(y)),
            28,
            4
        )

#-----------------------------------------------
#reset simulation
def reset_simulation():
    global x,y, theta
    global previous_distance_error
    global simulation_time
    global current_linear_velocity
    global final_time

    simulation_time = 0.0
    current_linear_velocity = 0.0
    final_time = None
    
    #reset robot position
    x = 150.0
    y = 450.0

    #reset robot orientation
    theta = math.radians(-20)

    #reset previous distance error
    previous_distance_error = None

    #clear old trajectory
    trajectory.clear()

    #add starting position
    trajectory.append(
        (int(x), int(y))
    )

#-----------------------------------------------
#draw information text on the screen
def draw_information():
    (
        error_x,
        error_y,
        distance_error,
        desired_heading,
        heading_error
    ) = calculate_errors()

    #display information on the screen
    information = [
        f"Controller Mode: {controller_mode}",
        f"Linear Velocity: {current_linear_velocity:.2f}",
        f"Simulation Time: {simulation_time:.2f} s",
        f"Robot Position: ({x:.1f}, {y:.1f})",
        f"Target Position: ({traget_x}, {traget_y})",
        f"X Error (ex): {error_x:.2f}",
        f"Y Error (ey): {error_y:.2f}",
        f"Distance Error (ed): {distance_error:.2f}",
        f"Current Heading (theta): {math.degrees(theta):.2f}°",
        f"Desired Heading (theta_d): {math.degrees(desired_heading):.2f}°",
        f"Heading Error (e_theta): {math.degrees(heading_error):.2f}°"
    ]

    if final_time is not None:
        information.append(
            f"Target Reached Time: {final_time:.2f} s"
        )
        
    y_position = 20

    for line in information:
        text = font.render(line, True, BLACK)
        screen.blit(text, (20, y_position))
        y_position += 28

#-----------------------------------------------
#main simulation loop
running = True


while running:

    #check pygame events
    for event in pygame.event.get():
        #close window 
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            #key 1=p control
            if event.key == pygame.K_1:
                controller_mode="P"
                reset_simulation()
            #key 2 = PD control
            elif event.key == pygame.K_2:
                controller_mode = "PD"
                reset_simulation()

    #update robot using p controller
    if controller_mode == "P":
        p_control_update()
    elif controller_mode == "PD":
        pd_control_update()


    #background color
    screen.fill((245,245,245))

    draw_trajectory()
    draw_target()
    draw_robot()
    draw_final_position()
    draw_information()
    
    #update display
    pygame.display.flip()

    #limit simulation to 60 frames per second
    clock.tick(60)

#close pygame
pygame.quit()

