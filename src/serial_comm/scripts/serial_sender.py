#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys

def serial_sender():
    rospy.init_node('serial_sender', anonymous=True)
    pub = rospy.Publisher('/serial/send', String, queue_size=10)
    
    rate = rospy.Rate(10)  # 10hz
    
    print("串口发送测试程序")
    print("输入要发送的数据，按回车发送，输入'quit'退出")
    
    while not rospy.is_shutdown():
        try:
            user_input = input(">>> ")
            if user_input.lower() == 'quit':
                break
            
            msg = String()
            msg.data = user_input + '\n'  # 添加换行符
            pub.publish(msg)
            rospy.loginfo("已发送: %s", user_input)
            
        except KeyboardInterrupt:
            break
        except EOFError:
            break
    
    print("程序退出")

if __name__ == '__main__':
    try:
        serial_sender()
    except rospy.ROSInterruptException:
        pass
