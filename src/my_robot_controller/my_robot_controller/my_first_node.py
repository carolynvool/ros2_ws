#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self): ## constructor of the class
        super().__init__("first_node") # initiated name
        self._counter=0
        
        self.create_timer(1,self.timer_callback)

    def timer_callback(self): # timer will react based on the messaage
        self.get_logger().info("hello "+str(self._counter))
        self._counter+=1





def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node) # since in the instruction we have a callback, it spins the node
    rclpy.shutdown()









if __name__=='__main__':
    main()