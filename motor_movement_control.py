"""
This code executes a complete cycle for a robotic arm, handling the initialization, catching, 
and releasing of an object while visualizing the arm's configuration.

1. **Motor Control**: Manages six motors with specified positions for catching and releasing objects, 
     with gradual movement implementation for smooth operation.
2. **Inverse Kinematics**: Calculates required angles based on target coordinates for accurate positioning.
3. **PWM Conversion**: Converts calculated angles to PWM signals for motor movement.
4. **Safety Initialization**: Moves motors to a safe starting position before operations.
5. **Gripper Action**: Controls the gripper for catching and releasing tasks.
6. **Visualization**: Uses Matplotlib to display the robotic arm's link positions and pose during operation.
7. **Logging**: Writes PWM values and operation orders to a file for tracking movements.

   Complete movement steps: 
        initialize_safety_positionssafety_positions 
    => move_to_intermediate_position => Move to Target Point => Catch 
    => move_to_intermediate_position => Move to Release Point => Release
    
"""
import sys
import os
import inverse_kinematics_calculations as iv
sys.path.append('.')
from coordinate_conversion import final_coord
          
# Motor Information
#             #NO,  left(0),  45,  90,  135, right(180),init, cur  
              #     open                      close
              #0     1       2     3    4      5         6    7     
motor_info =[[ 0,    45,     65,  95,  120,   147,      95,   95],                           #Motor 0 : Base
			 [ 2,   140,    115,  80,   50,    26,      26,   26],                           #Motor 2
			 [ 3,    90,     90,  65,   40,    20,      20,   20],                           #Motor 3
             [ 5,   105,    105,  80,   55,    25,      25,   25],                           #Motor 5
             [ 7,    70,     70,  70,   70,    115,     115,  115],
             [ 4,    100,   100,  100,  100,   100,     100,  100]]                          #Motor 7
motor_names = ['Motor_0(PWM)', 'Motor_2(PWM)', 'Motor_3(PWM)', 'Motor_5(PWM)', 'Motor_7(PWM)', 'Motor_4(PWM)']
#Aim Point             #Catch      #Release
                  #No  #Ang #PWM  
                  #0   1      2    3    4
motor_ang_info = [[0,  0,     0,   0,   0],
                  [2,  0,     0,   0,   0],
                  [3,  0,     0,   0,   0],
                  [5,  0,     0,   0,   0]]

#Convert real angle to motor PWM value
def convert_angle_to_pwm(Motor_No, Angle):
    global angle_m

    #Define the angle range and corresponding values for each motor.
    motor_ranges = {
        0: [(0, 45, 45, 0), (45, 90, 65, 45), (90, 135, 95, 90), (135, 180, 120, 135)],
        1: [(0, 45, 140, 0), (45, 90, 115, 45), (90, 135, 80, 90), (135, 180, 50, 135)],
        2: [(45, 90, 90, 45), (90, 135, 65, 90), (135, 180, 40, 135)],
        3: [(45, 90, 105, 45), (90, 135, 80, 90), (135, 180, 55, 135)],
    }

    if Motor_No not in motor_ranges:
        raise ValueError(f"Invalid Motor_No: {Motor_No}")

    for low, high, value, base in motor_ranges[Motor_No]:
        if low < Angle <= high:
            rate = (value - base) / (high - low)
            angle_m = (Angle - low) * rate + base
            return angle_m

    raise ValueError(f"Angle {Angle} out of range for motor {Motor_No}")

# Convert multiple motor angles to PWM simultaneously
def convert_all_angles_to_pwm_To_Motor(motor0,motor2,motor3,motor5):
    PWM0 = convert_angle_to_pwm(0,motor0)
    PWM2 = convert_angle_to_pwm(2,motor2)
    PWM3 = convert_angle_to_pwm(3,motor3)
    PWM5 = convert_angle_to_pwm(5,motor5)
    return PWM0,PWM2,PWM3,PWM5

# Simulate motor movement mode by sending PWM values (+1 or -1) to motors, making them gradually move
def move_motor(Motor_No, Motor_Angle, mode):	
    # Adjust the motor number to the array index, Motor No refer to motor_info[]
    if Motor_No == 2: Motor_No = 1
    if Motor_No == 3: Motor_No = 2
    if Motor_No == 5: Motor_No = 3
    if Motor_No == 7: Motor_No = 4
    
    # check angle not out the limit
    if Motor_Angle < motor_info[Motor_No][1] : Motor_Angle = motor_info[Motor_No][1]
    if Motor_Angle > motor_info[Motor_No][5] : Motor_Angle = motor_info[Motor_No][5]

    #Current
    if mode == 'c': m = 7
    #Initial
    elif mode == 'i': m = 6
    
    # Move the motor to the target angle step by step，if it's Motor 5, move inversely
    while motor_info[Motor_No][m] != Motor_Angle:  
        if motor_info[Motor_No][m] < Motor_Angle:  
            motor_info[Motor_No][m] += 1  
            if Motor_No == 5:  
                motor_info[3][m] -= 1  

        elif motor_info[Motor_No][m] > Motor_Angle:  
            motor_info[Motor_No][m] -= 1 
            if Motor_No == 5: 
                motor_info[3][m] += 1  

# Set initial safety values before activating the robotic arm          
def initialize_safety_positions(On):
    if On == 1 :
        print("\n\n") 

        initial_pwm = [motor_info[i][6] for i in range(5)] + [100]  # Collect initial PWM values
        print('Initial Motor Names: ', motor_names)
        print('Initial PWM Values: ', initial_pwm)

        # Move motors to the initial safe position
        positions = [95, 26, 20, 25, 115]  # Corresponding positions for each motor
        for i, pos in enumerate(positions):
            move_motor(i * 2, pos, 'i')  # Move motor at index i * 2 to the specified position

        for i in range(5):
            motor_info[i][7] = motor_info[i][6]

        print("------------------------------------------------------------------------------------------------")
        current_pwm = [motor_info[i][7] for i in range(5)] + [100]
        print('Current Motor Names: ', motor_names)
        print('Current PWM Values: ', current_pwm)

#Middle Catch : is the intermediate point the robotic arm passes between the initial catch and the release.        
def move_to_intermediate_position(On,Motor_PWM_0,name,M7):
    if On == 1 :

        # Move motors to the specified positions
        motor_positions = [Motor_PWM_0, 80, 65, 25, M7]
        for i, pos in enumerate(motor_positions):
            move_motor(i * 2, pos, 'c')  # Move motors 0, 2, 3, 5, and 7

        # Update motor_info for the new positions
        motor_info[0][7] = Motor_PWM_0 
        motor_info[1][7] = 80
        motor_info[2][7] = 65
        motor_info[3][7] = 25
        motor_info[4][7] = M7

        print("-----------------------------------------------------------------------------------------------") 
        
        # Collect the PWM values
        middle_list = [motor_info[i][7] for i in range(5)] + [100]  
        print(f"{name} : {motor_names}") 
        print(f"{name} : {middle_list}")

#Move all motors sequentially to the specified positions in the following order
#starting from the motor closest to the robot base.
def move_all_motors(Motor_Angle_0,Motor_Angle_2,Motor_Angle_3,Motor_Angle_5,m7):
    move_motor(0, Motor_Angle_0, 'c')
    move_motor(2, Motor_Angle_2, 'c')
    move_motor(3, Motor_Angle_3, 'c')
    move_motor(5, Motor_Angle_5, 'c')

    if m7 == 'o':
        move_motor(7, 70,'c')
        m7 = 70 #
    if m7 == 'c':
        move_motor(7, 115,'c')
        m7 = 115#

    temp = [Motor_Angle_0,Motor_Angle_2,Motor_Angle_3,Motor_Angle_5,m7]
    for i in range(5):
        motor_info[i][7] = temp[i] 
    print("---------------------------------------------------------------------------------------------------")

    Current_PWM = [motor_info[0][7],motor_info[1][7],motor_info[2][7],motor_info[3][7],motor_info[4][7],100]
    print('Current_name : ',motor_names)
    print('Current_PWM : ',Current_PWM)

    """Control the gripper (open/close) using Motor7. 
    使用示例
        gripper_action('catch', 1)  # 抓取
        gripper_action('release', 1)  # 釋放
    """

def gripper_action(action, On):
    if On == 1:
        if action == 'catch':
            print("------------------------------------------CATCH------------------------------------------------")
            move_motor(7, 115, 'c')
            motor_info[4][7] = 115
        elif action == 'release':
            print("------------------------------------------RELEASE----------------------------------------------")
            move_motor(7, 70, 'c')
            motor_info[4][7] = 70
        else:
            print("Invalid action. Please use 'catch' or 'release'.")
        
        Current_PWM = [motor_info[0][7], motor_info[1][7], motor_info[2][7], motor_info[3][7], motor_info[4][7], 100]
        print('Current_name : ', motor_names)
        print('Current_PWM : ', Current_PWM)

#Inverse Kinematics:Given the target position coordinates, calculate the required angles.   
def calculate_angles_for_target(X,Y,Z,n,name): #Catch Point [][n=1](Ang)  n=2(PWM) Release Point [][n=3](Ang) n=4(PWM)
    print(" ")
    print(name,":"," X = ",X,"Y =", Y,"Z =",Z)

    motor_ang = iv.Inv_Kine_All(X,Y,Z) 
    motor_ang = [motor_ang[0],motor_ang[1],motor_ang[2],motor_ang[3]]
    update_motor_angles(motor_ang[0],motor_ang[1],motor_ang[2],motor_ang[3],n) 

    ALL = convert_all_angles_to_pwm_To_Motor(motor_ang_info[0][n],motor_ang_info[1][n],motor_ang_info[2][n],motor_ang_info[3][n])
    ALL =[ALL[0],ALL[1],ALL[2],ALL[3]]
    for i in range (4):
        motor_ang_info[i][n+1] = ALL[i]

    Aim_Poi_name  = ['Motor_0(PWM)','Motor_2(PWM)','Motor_3(PWM)','Motor_5(PWM)']
    Aim_Poi_list = [motor_ang_info[0][n+1],motor_ang_info[1][n+1],motor_ang_info[2][n+1],motor_ang_info[3][n+1]]
    print(name," : ",Aim_Poi_name)
    print(name," : ",Aim_Poi_list)
    
#Write Aim Angle to motor_info as Global Variable
#Store the calculated inverse kinematics angles for each axis of the target position in a 2D list called motor_ang_info.
def update_motor_angles(motor0,motor2,motor3,motor5,p):
    motor = [motor0,motor2,motor3,motor5]
    for i in range (4):
        motor_ang_info[i][p] = motor[i]
    return motor_ang_info[0][p],motor_ang_info[1][p],motor_ang_info[2][p],motor_ang_info[3][p]

#Given the coordinates for releasing  an object, use the inverse kinematics program to calculate the required angles.
def calculate_release_anglesnt(On):
    if On == 1:
        X=10
        Y=-18
        Z=27
        calculate_angles_for_target(X,Y,Z,3,'Release Point')

def convert_all_angles_to_pwm_To_M(On):
    if On == 1:
        Motor_PWM = []
        Motor_PWM[0] = convert_angle_to_pwm(0,motor_ang_info[0][1])
        Motor_PWM[1] = convert_angle_to_pwm(0,motor_ang_info[1][1])
        Motor_PWM[2] = convert_angle_to_pwm(0,motor_ang_info[2][1])
        Motor_PWM[3] = convert_angle_to_pwm(0,motor_ang_info[3][1])

#Write the PWM values for each step motor to pwm.txt.
def write_pwm_to_file(order):
    motor_info_Write = [str(order),0,0,0,0,0,0]
    for i in range(6):
            motor_info_Write[i+1] = ','+str(motor_info[i][7]) 
    with open('C:\\Users\\a12\\Desktop\\YOLO_V5\\pwm.txt','a') as f:
        f.writelines(motor_info_Write)
        f.write('\n')
    
if __name__ == "__main__":
    initialize_safety_positions(1)
    write_pwm_to_file(1)

    #Move Front To Aim Point      
    X=final_coord[0]   
    #fixed because the object is on the table -20+2.8-1 = -16.2        
    Y=-16.2               
    Z=final_coord[1]

    calculate_angles_for_target(X,Y,Z,1,'Object Point')
    #Middle
    move_to_intermediate_position(1,motor_ang_info[0][2],'Front',motor_info[4][7])    
    write_pwm_to_file(2)
    move_all_motors(motor_ang_info[0][2],motor_ang_info[1][2],motor_ang_info[2][2],motor_ang_info[3][2],'o')   
    write_pwm_to_file(3)
    #Catch
    gripper_action('catch', 1)
    write_pwm_to_file(4)

    #Move Back Middle
    move_to_intermediate_position(1,motor_ang_info[0][2],'Back',motor_info[4][7])
    write_pwm_to_file(5)
    #Calculate Release Point
    calculate_release_anglesnt(1)
    move_all_motors(motor_ang_info[0][4],motor_ang_info[1][4],motor_ang_info[2][4],motor_ang_info[3][4],'c')  
    write_pwm_to_file(6)
    gripper_action('release', 1)
    write_pwm_to_file(7)
    #Back to start position
    initialize_safety_positions(1)
    write_pwm_to_file(8)
    print("___________________________________F   I   N   I   S   H_________________________________________") 
    
