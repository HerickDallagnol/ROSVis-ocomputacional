#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Point 
import time

target_coordinates = None
robot_pose = None

def target_callback(msg):
    global target_coordinates
    target_coordinates = msg

def robot_pose_callback(msg):
    global robot_pose
    robot_pose = msg.pose  

def move_to_target():
    global target_coordinates, robot_pose
    rospy.init_node('game_control', anonymous=True)
    
    #TOPICOS
    rospy.Subscriber('/target_coordinates', Point, target_callback)
    rospy.Subscriber('/game/pose', PoseStamped, robot_pose_callback)  

    joy_pub = rospy.Publisher('/game/joy', Joy, queue_size=10)

    #2DUMMY
    joy_msg = Joy()
    joy_msg.axes = [0.0, 0.0]
    joy_msg.buttons = [0] 
    joy_pub.publish(joy_msg)
    rospy.loginfo("DUMMY 1")
    time.sleep(2)
    
    joy_msg.axes = [0.0, 0.0]
    joy_msg.buttons = [0]
    joy_pub.publish(joy_msg)
    rospy.loginfo("DUMMY 2")
    time.sleep(2)

    rate = rospy.Rate(10) #tem q ver se não esta interferindo na resposta (acho que não)

    while not rospy.is_shutdown():
        if target_coordinates and robot_pose:
            # calcula a diferença 
            delta_x = target_coordinates.x - robot_pose.position.x
            delta_y = target_coordinates.y - robot_pose.position.y

            if delta_x > 0.1:  #se o alvo estiver a direita
                joy_msg.axes = [1.0, 0.0]  
            elif delta_x < -0.1:  #se o alvo estiver a esquerda
                joy_msg.axes = [-1.0, 0.0]  
            else:
                joy_msg.axes[0] = 0.0 
            
            if delta_y > 0.1:  #se o alvo estiver pra cima
                joy_msg.axes[1] = 1.0  
            elif delta_y < -0.1:  #o alvo estiver embaixo
                joy_msg.axes[1] = -1.0 
            else:
                joy_msg.axes[1] = 0.0  
            
            joy_pub.publish(joy_msg) 
            rospy.loginfo(f"Movendo para: ({target_coordinates.x}, {target_coordinates.y}) | Posi atual: ({robot_pose.position.x}, {robot_pose.position.y})")

        rate.sleep()

if __name__ == '__main__':
    try:
        move_to_target()
    except rospy.ROSInterruptException:
        pass
