# 2024 IESL RoboGames Competion 
# Round 01 COMPLETION - University Category Task

import math
from controller import Robot, DistanceSensor, Motor, Gyro

if __name__ == "__main__":
    
    TIME_STEP = 32
    MAX_SPEED = 6.28
    prevError = 0.0
    errorSum = 0.0
    integral_LIMIT = 100
    
    robot = Robot()  # Robot instance
       
    #    ** Initialize Devices **
    # -------------------------------
    # enable Camera
    camera = robot.getDevice('camera')
    camera.enable(TIME_STEP)

    # Define colors  
    red = "#FF0000"
    yellow = "#FFFF00"
    pink = "#FF00FF"
    brown = "#A5691E" 
    green = "#00FF00"
    
    # Define color order
    color_order = [red, yellow, pink, brown, green]
    index = [None] * len(color_order)  
    
    # Enable position snsors (type Sonar)
    ps = []
    psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    
    for i in range(8):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(TIME_STEP)  
   
    # enable Gyroscope
    gyro = robot.getDevice('gyro')
    gyro.enable(TIME_STEP)
   
    # enable Motors      
    L_Motor = robot.getDevice('left wheel motor')
    R_Motor = robot.getDevice('right wheel motor')
    L_Motor.setPosition(float('inf'))
    R_Motor.setPosition(float('inf'))
    L_Motor.setVelocity(0.0)
    R_Motor.setVelocity(0.0)
    
    # enable Encoders
    R_Encoder = robot.getDevice("right wheel sensor")
    L_Encoder = robot.getDevice("left wheel sensor")
    R_Encoder.enable(TIME_STEP)
    L_Encoder.enable(TIME_STEP)
    
         
    # Motor stop 
    def Motor_stop():
        L_Motor.setVelocity(0)
        R_Motor.setVelocity(0)    
    
      
    # Turn Right degrees - using Gyroscope           
    def turnR(degree):
        degree_rads = math.radians(degree)  # Convert degrees to radians
        current_angle = 0.0
        TIME_STEP_SEC = TIME_STEP / 10000  
    
        while robot.step(TIME_STEP) != -1:
            angular_velocity_z = gyro.getValues()[2]
            bias_angular_velocity_z = angular_velocity_z * 0.003  # Apply scale factor from gyro bias LooupTable
    
            # Integrate angular velocity to get the total angle
            current_angle += bias_angular_velocity_z * TIME_STEP_SEC    #TIME_STEP_SEC # 0.00143
            #print(f"Target degree angle rads: {degree_rads}, Current: {current_angle}")
    
            if abs(current_angle) >= abs(degree_rads):
                Motor_stop()
                break
            # Set motor speeds for turning
            L_Motor.setVelocity(0.3 * MAX_SPEED)
            R_Motor.setVelocity(-0.3 * MAX_SPEED)
        
        
    # Turn left degrees - using gyroscope           
    def turnL(degree):
        degree_rads = math.radians(degree)
        current_angle = 0.0
        TIME_STEP_SEC = TIME_STEP / 10000

        while robot.step(TIME_STEP) != -1:
            angular_velocity_z = gyro.getValues()[2]
            bias_angular_velocity_z = angular_velocity_z * TIME_STEP_SEC # 0.003  
        
            current_angle += bias_angular_velocity_z * TIME_STEP_SEC #TIME_STEP_SEC # 0.00143
            #print(f"Target degree angle rads: {degree_rads}, Current: {current_angle}")
        
            if abs(current_angle) >= abs(degree_rads):
                Motor_stop()
                break
            L_Motor.setVelocity(-0.3 * MAX_SPEED)
            R_Motor.setVelocity(0.3 * MAX_SPEED)
            
                   
    # Move Robot forward a bit - using Encoders
    def move(distance):
        # Robot parameters
        wheel_radius = 0.02  # Wheel radius (meters)
        wheel_circumference = 2 * math.pi * wheel_radius   # wheel circumference  
          
        target_rotation = (distance / wheel_circumference) * 2 * math.pi
        
        robot.step(TIME_STEP)
        initial_r_position = R_Encoder.getValue()
        initial_l_position = L_Encoder.getValue()
               
        while True:   
            robot.step(TIME_STEP)
            current_r_position = R_Encoder.getValue()
            current_l_position = L_Encoder.getValue()
        
            # Calculate wheel rotation
            r_travel = abs(current_r_position - initial_r_position)
            l_travel = abs(current_l_position - initial_l_position)
        
            # Stop when target rotation reached 
            if (r_travel >= target_rotation - 0.01) or (l_travel >= target_rotation - 0.01):
                Motor_stop()
                break            
            R_Motor.setVelocity(MAX_SPEED*0.5)
            L_Motor.setVelocity(MAX_SPEED*0.5) 
   
   
    # Color detection range
    def color_check(r, g, b):
        if (r < 100  and g < 20 and b < 20):
            return (255, 0, 0)    # red
            
        elif (r > 180 and g > 180 and b < 40):
            return (255, 255, 0)  # yellow
            
        elif (r > 180 and g < 40 and  b > 180):
            return (255, 0, 255)  # pink
            
        elif ((140 < r < 170) and (90 < g < 120) and b < 60):
            return (165, 105, 30) # brown
        
        elif (r < 100 and g > 180 and b < 100):
            return (0, 255, 0)    # green    
        return (r, g, b)  # if nothing matched return same RGB 
    
    # Refresh PS sensor array
    def read_sensors():   
        return [ps[i].getValue() for i in range(8)]
    
 
    # ================================================================================== 
       
    # MAIN LOOP
    while robot.step(TIME_STEP) != -1:

        #  *** COLOR DETECTION ***
        # -----------------------       

        # Get camera image
        image = camera.getImageArray()  # Get image RGB pixels
        width = camera.getWidth()       # 52 pixels
        height = camera.getHeight()     # 39 pixels
        # Create pixel array
        if image:
            pixels = []
            for x in range(width):
                row = []
                for y in range(height):
                    r = image[x][y][0] # Red
                    g = image[x][y][1] # Green
                    b = image[x][y][2] # Blue 
                    r, g, b = color_check(r, g, b)
                    row.append((r, g, b))
                pixels.append(row)
   
           #                \\*** HOW PIXELS ARRAY IS STRUCTURED ***//
           # pixels[0] = ['#00FF00', '#00FF00', ..., '#00FF00']    # 39 values (column 0)
           # pixels[1] = ['#00FF00', '#00FF00', ..., '#00FF00']    # 39 values (column 1)
           # ...
           # pixels[52] = ['#00FF00', '#00FF00', ..., '#00FF00']  # 39 values (column 127)
            
        # Filter necessary region from pixels array
        #middle_region = [pixels[col][16:24] for col in range(21, 31)] #  10x10 middle region
        upper_half = [col[:20] for col in pixels] 
        #middle_pixel = pixels[int(width / 2)][int(height / 2)] # [col 26][row 19]
        
        # Extract brightest pixel from filtered region
        max_RGB = max([max(row) for row in upper_half])  # Select max RGB tupple from upper_half array
        #print(max_RGB)
        
        # Convert RGB to hexadecimal
        hex_color = f"#{max_RGB[0]:02X}{max_RGB[1]:02X}{max_RGB[2]:02X}"
        #print(hex_color)
        
        # Detection order setup
        if index == color_order:
            print(f"{index} -- Color pattern matched! ** GOAL ACHIEVED **")
            Motor_stop()
            TIME_STEP = -1 # Program stop
                 
        elif color_order[0] == hex_color: # initial color match
            index[0] = hex_color   
        else:     
            for i in range(1, len(color_order)):
                if index[i-1] == color_order[i-1] and color_order[i] == hex_color:
                    index[i] = hex_color 
        print(" ")   
        print(f"Color Pattern : {index}")
            

        #  *** NAVIGATION ***  
        # --------------------- 
        
        # Read PS sensors 
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
    
        Kp = 0.01  # 0.01 optimal
        Ki = 0.005 # 0.005 is optimal. Higher, less turn curve, but not fit for all states
        Kd = 0.07  # 0.07 optimal
        set_distance_R = 100
        set_distance_L = 100
        base_speed = MAX_SPEED * 0.7
        
        # Wall detection
        front_wall = psValues[0] > 100 or psValues[7] > 100  
        right_90 = psValues[2] > 95  
        left_90 =  psValues[5] > 95
        
        right_distance = max(psValues[2], psValues[1])
        left_distance = psValues[5]
        
       # PID controller 
       # Get error
        error = set_distance_R - right_distance # Right wall prioritized
        """
        if left_90 and right_90:
            error1 = set_distance_R - right_distance 
            error2 = set_distance_L - left_distance
            error = (1.0 * error1) + (0.5 * error2)   # Right wall prioritized, Right err + left err
        """
        # Kp
        proportional = Kp * error
 
        # Ki
        errorSum += error
        errorSum = max(min(errorSum, integral_LIMIT), - integral_LIMIT)  # Prevent integral wind-up
        integral = Ki * errorSum
        
        # Kd
        derivitive = Kd * (error - prevError)
        prevError = error
        
        pid = proportional + integral + derivitive
 
        left_speed = base_speed + pid
        right_speed = base_speed - pid
        
        # Scale speeds if necessary to fit base_speed range
        max_wheel_speed = max(abs(left_speed), abs(right_speed))
        if max_wheel_speed > base_speed:
            scale_factor = base_speed / max_wheel_speed
            left_speed *= scale_factor
            right_speed *= scale_factor
        
        if front_wall: 
            Motor_stop()
            while front_wall:
                turnL(20) # 20 / 30 # Turn left degree
                robot.step(TIME_STEP)  # Refresh robot
                psValues = read_sensors()
                front_wall = psValues[0] > 100 or psValues[7] > 100 
                      
        else:
            robot.step(TIME_STEP) # Refrsh robot
            psValues = read_sensors()
            front_wall = psValues[0] > 100 or psValues[7] > 100 
            
            if not front_wall:
                L_Motor.setVelocity(left_speed)
                R_Motor.setVelocity(right_speed)
  