"""
Inverse Kinematics for Robotic Arm with Visualization

Calculates the angles for a planar robotic arm to reach a target position (X, Y) and the Z axis. 
Visualizes the arm's configuration using matplotlib. 
This file is imported into motor_movement_control.py.
"""

from cmath import pi
import math
import matplotlib.pyplot as plt

def inverse_kinematics(X, Y):
    L1 = 22               #28
    L2 = 20               #22
    L3 = 19               #18
    
    X2 = X
    Y2 = Y + L3
    
    L_base = math.sqrt((X2 * X2) + (Y2 * Y2))
    
    phi = math.acos(((L_base * L_base) + (L1 * L1) - (L2 * L2)) / (2 * L_base * L1)) * 180 / pi     
    f_base = (math.atan(-Y2 / X2)) * 180 / pi 
    f12 = math.acos(((L1 * L1) + (L2 * L2) - (L_base * L_base)) / (2 * L1 * L2)) * 180 / pi   
    f1 = phi - f_base
    f2 = 180 - f12                                                               
    base_2_Ang = f12 - (90 - f1) - 90 # f1
    base_2_rad = base_2_Ang * pi / 180

    X0 = 0
    Y0 = 0
    X3 = X
    Y3 = Y

    X1 = L1 * math.cos(f1 * pi / 180)
    Y1 = L1 * math.sin(f1 * pi / 180)

    len_p3_p1 = math.sqrt(((X - X1) * (X - X1) + (Y - Y1) * (Y - Y1)))
    f23 = math.acos(((L1 * L1) + (L2 * L2) - (len_p3_p1 * len_p3_p1)) / (2 * L1 * L2)) * 180 / pi     
    
    X_link1 = [X0,X1]
    Y_link1 = [Y0,Y1]
    X_link2 = [X1,X2]
    Y_link2 = [Y1,Y2]
    X_link3 = [X2,X3]
    Y_link3 = [Y2,Y3]
    #931 lab ppt17
    plt.annotate("origin",(X0,Y0))
    plt.annotate("point1",(X1,Y1))
    plt.annotate("point2",(X2,Y2))
    plt.annotate("point3",(X3,Y3))
    plt.plot(X_link1, Y_link1, label="link1",color='orange',linestyle='solid')
    plt.plot(X_link2, Y_link2, label="link2",color='blue',linestyle='solid')
    plt.plot(X_link3, Y_link3, label="link3",color='green',linestyle='solid')
    plt.title('Simulation')
    plt.grid(linestyle = 'solid',color = 'graY')
    plt.xlim(-8.6,40)
    plt.ylim(-50,50)
    plt.xlabel('X Axis',fontsize = 15)
    plt.ylabel('Y Axis',fontsize = 15)
    plt.show()

    return f1, f2, f23

"""
    Calculate the relationship between Z and X in a plane.
    
    :param flat_Z: Z coordinate
    :param flat_X: X coordinate
    :return: Calculated angle
"""
def inverse_kinematics_flat(flat_Z, flat_X):
    flat_Z = float(flat_Z)
    flat_X = float(flat_X)
    Ang = (math.atan(flat_Z / flat_X)) * 180 / math.pi
    
    if flat_Z < 0:
        Ang = 90 + (Ang * (-1))
    else:
        Ang = 90 - Ang

    return Ang

"""
    Calculate all relevant angles in a single function.
    
    :param X: X coordinate of the target position
    :param Y: Y coordinate of the target position
    :param Z: Z coordinate of the target position
    :return: Angles for four motors
"""
def calculate_all_angles(X, Y, Z):
    Ang_XY = inverse_kinematics(X, Y)
    Ang_XZ = inverse_kinematics_flat(Z, X)

    return Ang_XZ, Ang_XY[0], Ang_XY[1], Ang_XY[2]

if __name__ == "__main__":
    X = 20           # X=20,Y=-20,Z=30
    Y = -20          # fixed because the object is on the table
    Z = 30
    
    motor_ang = calculate_all_angles(X, Y, Z) 
    motor_ang = [motor_ang[0], motor_ang[1], motor_ang[2], motor_ang[3]]   
    
    print("List : ", motor_ang)
    print('f_base = ', motor_ang[0])
    print('f1 = ', motor_ang[1])
    print('f3 = ', motor_ang[2])
    print('f5 = ', motor_ang[3])





	
	
