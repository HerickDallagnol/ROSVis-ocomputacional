#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import time

def move_joy():
    rospy.init_node('game_control', anonymous=True)
    joy_pub = rospy.Publisher('/game/joy', Joy, queue_size=10)
    
    joy_msg = Joy()
    joy_pub.publish(joy_msg)

    # dummy
    joy_msg.axes = [0.0, 0.0]
    joy_msg.buttons = [0]  # Solta o botão
    joy_pub.publish(joy_msg)
    rospy.loginfo("DUMMY")

    #dummy
    joy_msg.axes = [0.0, 0.0]  
    joy_msg.buttons = [0]  
    joy_pub.publish(joy_msg)
    rospy.loginfo("dummy")
    time.sleep(5)

    # Parado 
    joy_msg.axes = [-1.0, 0.0]
    joy_msg.buttons = [0]  # Solta o botão
    joy_pub.publish(joy_msg)
    rospy.loginfo("esquerda")

    # Direita -
    joy_msg.axes = [1.0, 0.0] 
    joy_msg.buttons = [1]  
    joy_pub.publish(joy_msg)
    rospy.loginfo("direita")
    time.sleep(5)

     # Esquerda 
    joy_msg.axes = [-1.0, 0.0]  
    joy_msg.buttons = [0]  
    joy_pub.publish(joy_msg)
    rospy.loginfo("esquerda")
    time.sleep(5)

    # Direita
    joy_msg.axes = [1.0, 0.0]  
    joy_msg.buttons = [1]  
    joy_pub.publish(joy_msg)
    rospy.loginfo("direita")
    time.sleep(5)

    # Direita 
    joy_msg.axes = [0.0, 1.0]  
    joy_msg.buttons = [1] 
    joy_pub.publish(joy_msg)
    rospy.loginfo("cima")
    time.sleep(5)

    # Direita 
    joy_msg.axes = [0.0, -1.0]  
    joy_msg.buttons = [1]  
    joy_pub.publish(joy_msg)
    rospy.loginfo("baixo")
    time.sleep(5)

     # Direita 
    joy_msg.axes = [-1.0, 1.0] 
    joy_msg.buttons = [1] 
    joy_pub.publish(joy_msg)
    rospy.loginfo("diagonal 1")
    time.sleep(5)

      # Direita
    joy_msg.axes = [1.0, 1.0] 
    joy_msg.buttons = [1] 
    joy_pub.publish(joy_msg)
    rospy.loginfo("diagonal 2")
    time.sleep(5)

     # Direita 
    joy_msg.axes = [0.0, 0.0]  
    joy_msg.buttons = [1]  
    joy_pub.publish(joy_msg)
    rospy.loginfo("diagonal 2")
    time.sleep(5)

if __name__ == '__main__':
    try:
        move_joy()
    except rospy.ROSInterruptException:
        pass
