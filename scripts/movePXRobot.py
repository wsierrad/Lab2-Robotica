"""
Allows to use the service dynamixel_command 
"""
from turtle import pos
from matplotlib.pyplot import switch_backend
# import swift
import rospy
import time
import termios, os, sys
# from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

__author__ = "F Gonzalez, S Realpe, JM Fajardo"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]
__email__ = "fegonzalezro@unal.edu.co"
__status__ = "Test"

TERMIOS = termios
home_pos = 512
obj_pos_waist = 820
obj_pos_shoulder = 650
obj_pos_elbow = 300
obj_pos_wrist = 204
pos_obj = [obj_pos_waist, obj_pos_shoulder, obj_pos_elbow, obj_pos_wrist]
name = ['waist','shoulder','elbow','wrist']
id = 1
change = 0

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

#Se hace uso del código brindado para la detección de las teclas oprimidas
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

if __name__ == '__main__':
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        # map(0,1023,-150,150)
        pos = home_pos
        jointCommand('', 1, 'Torque_Limit', 600, 0)
        jointCommand('', 2, 'Torque_Limit', 500, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
        while(1):
            key = getkey()
            if key == b'w':
                if id != 4:
                    id =  id + 1
                else:
                    id = 1
            if key == b's':
                if id != 1:
                    id = id - 1
                else:
                    id = 4
            if key == b'd':
                pos = pos_obj[id-1]
                change = 1
            if key == b'a':
                pos = home_pos
                change = 1
            print('Junta: ' + name[id-1] + str(id))
            print('Posicion: ' + str(pos))
            if change == 1:
                jointCommand('', id, 'Goal_Position',pos, 0.5)
                time.sleep(0.5)
                change = 0
    except rospy.ROSInterruptException:
        pass