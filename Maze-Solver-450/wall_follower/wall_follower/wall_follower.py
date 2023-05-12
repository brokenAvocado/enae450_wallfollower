"""
    Group 6: Wall Follower Code for Simulation
    Uses a wall following algorithm in order to solve complex mazes
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

class Follow(Node):
    def __init__(self):
        super().__init__('follow')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        self.subscription= self.create_subscription(
            LaserScan,
            '/scan',  ## Read
            self.listener_callback,
            qos_profile,
        )
        timer_period = 0.5  # seconds

        # NOTE Direction booleans are not used anymore
        # self.turnLeft = False
        # self.turnRight = False
        # self.turnSharpLeft = False
        # self.turnSharpRight = False

        self.min_front_dist = 0
        self.min_frontright_dist = 0
        self.min_backright_dist = 0
        self.min_frontleft_dist = 0
        self.min_backleft_dist = 0
        self.inTurn = False

        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
    def getch(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def timer_callback(self):
        '''
        Publisher callback function
        TODO: implement
        '''
        msg = Twist()
        # key = self.getch()

        ########### Gazebo vs Real World #################
        # comment out the one not in use

        # Real World
        realfactor = 0.14
        bound1 = 0.38-realfactor       # Side wall close threshold       
        bound2 = 0.34-realfactor       # Side wall too close threshold     
        bound3 = 0.38-realfactor       # Front wall threshold              
        bound4 = 0.40-realfactor       # Side wall far threshold          
        bound5 = 0.45-realfactor       # Back wall threshold               
        bound6 = 0.43-realfactor       # Side wall too far threshold 

        # Gazebo
        # bound1 = 0.38       # Side wall close threshold       
        # bound2 = 0.34       # Side wall too close threshold     
        # bound3 = 0.38       # Front wall threshold              
        # bound4 = 0.40       # Side wall far threshold          
        # bound5 = 0.45       # Back wall threshold               
        # bound6 = 0.43       # Side wall too far threshold  

        # bound1 = 0.40       # Side wall close threshold       
        # bound2 = 0.38       # Side wall too close threshold     
        # bound3 = 0.42       # Front wall threshold              
        # bound4 = 0.50       # Side wall far threshold          
        # bound5 = 0.55       # Back wall threshold               
        # bound6 = 0.55       # Side wall too far threshold  

        ###################################################

        slow_linear = -0.01     
        fast_linear = -0.12     
        fast_angular = 1.0     
        med_angular = 0.28      
        slow_angular = 0.15   

        # Corner/Intersection Turning 
        if self.min_front_dist < bound3: #and (self.min_backright_dist < bound5 or self.min_backleft_dist < bound5): #and not(self.inTurn):
            if self.min_backright_dist < bound5:
                # if self.min_backright_dist > bound4:
                #     msg.linear.x = slow_linear
                #     msg.angular.z = -1.0
                #     self.get_logger().info("sharp right")
                # if self.min_backright_dist < bound4:
                msg.linear.x = slow_linear
                msg.angular.z = 1.0
                self.get_logger().info("sharp left")
                self.inTurn = True
            # elif self.min_backleft_dist < bound5:
            #     msg.linear.x = slow_linear
            #     msg.angular.z = -1.0
            #     self.get_logger().info("sharp right")
                # self.inTurn = True
            else:
                msg.linear.x = fast_linear+0.03
                msg.angular.z = -(med_angular+0.3)
                self.get_logger().info("special right hehe")
        # Path Turning
        elif self.min_frontright_dist < bound1 and self.min_front_dist > bound3:
            self.inTurn = False
            if self.min_frontright_dist < bound2:
                # For left only: slow_linear and fast_angular works
                msg.linear.x = fast_linear
                msg.angular.z = med_angular 
                self.get_logger().info("med left")
            else:
                msg.linear.x = fast_linear
                msg.angular.z = slow_angular
                self.get_logger().info("left")
        elif self.min_frontright_dist > bound4 and self.min_front_dist > bound3:
            self.inTurn = False
            if self.min_frontright_dist > 0.35:
                msg.linear.x = fast_linear+0.03
                msg.angular.z = -(med_angular+0.3)
                self.get_logger().info("special right")
            elif self.min_frontright_dist > bound6:
                msg.linear.x = fast_linear
                msg.angular.z = -med_angular
                self.get_logger().info("med right")
            else:
                msg.linear.x = fast_linear
                msg.angular.z = -slow_angular
                self.get_logger().info("right")
        else:
            self.inTurn = False
            msg.linear.x = fast_linear
            msg.angular.z = 0.0
            self.get_logger().info("straight")


        # NOTE old code from previous approaches
        #----------------------------------------
        # if self.min_backright_dist < bound3 and self.min_backleft_dist < bound3:
            #     msg.linear.x = 0.0
            #     msg.angular.z = 1.5
            #     self.get_logger().info("turn around")

        # # Front right wall = close  &  Front wall = far
        # # ---> Fast straight
        # # ---> Slow left
        # if self.min_frontright_dist < bound1 and self.min_front_dist > bound3:
        #     msg.linear.x = fast_linear
        #     msg.angular.z = slow_angular
        #     # Front right wall = too close
        #     # ---> Medium left
        #     if self.min_frontright_dist < bound2:
        #         msg.angular.z = med_angular
        #     self.get_logger().info("left")
        # # Front left wall = close  &  Front wall = far
        # # ---> Fast straight
        # # ---> Slow right
        # # elif self.min_frontleft_dist < bound1 and self.min_front_dist > bound3: 
        # #     msg.linear.x = fast_linear
        # #     msg.angular.z = -slow_angular
        # #     # Front left wall = too close
        # #     # ---> Medium right
        # #     if self.min_frontleft_dist < bound2:
        # #         msg.angular.z = -med_angular
        # #     self.get_logger().info("right")
        # # Front wall = close
        # elif self.min_front_dist < bound3:
        #     # Back right = close  &  Back left = close
        #     # ---> Fast left
        #     # ---> Turn around
        #     if self.min_backright_dist < bound3 and self.min_backleft_dist < bound3:
        #         msg.linear.x = 0.0
        #         msg.angular.z = 1.5
        #         self.get_logger().info("turn around")
        #     # Back left closer than back right
        #     # ---> Fast right
        #     elif self.min_backleft_dist < self.min_backright_dist:
        #         msg.linear.x = fast_linear
        #         msg.angular.z = -1.0
        #         self.get_logger().info("sharp right")
        #     # Back right closer than back left
        #     # ---> Fast left
        #     elif self.min_backleft_dist > self.min_backright_dist:
        #         msg.linear.x = fast_linear
        #         msg.angular.z = 1.0
        #         self.get_logger().info("sharp left")
        # else:
        #     msg.linear.x = 0.08
        #     msg.angular.z = 0.0
        #     self.get_logger().info("straight")
        # # elif self.min_front_dist < 0.2 and self.min_right_dist < 0.2:
        # #     msg.linear.x = 0.0
        # #     msg.angular.z = -1.0

        self.publisher_.publish(msg)
    
    def listener_callback(self,msg):
        '''
        Subscription Callback 
        TODO: implement
        '''
        # self.get_logger().info(str(len(msg.ranges)))

        ########### Gazebo vs Real World #################
        # comment out the one not in use

        # Real World
        front = range(-15,15)
        back_right = range(507, 549)
        front_right = range(591,739)
        front_left = range(49,169)
        back_left = range(211,253)
        f = 0
        fl = 95
        l = 190
        r = 570
        fr = 665


        # Gazebo
        # front = range(-7,7)
        # back_right = range(240,260)
        # front_right = range(280,350)       
        # front_left = range(23,80)
        # back_left = range(100,120)
        # f = 0
        # fl = 45
        # l = 90
        # r = 270
        # fr = 315

        ##################################################

        self.min_front_dist = msg.ranges[f]
        for i in front:
            if msg.ranges[i] < self.min_front_dist:
                self.min_front_dist = msg.ranges[i]

        self.min_frontleft_dist = msg.ranges[fl]
        for i in front_left:
            if msg.ranges[i] < self.min_frontleft_dist:
                self.min_frontleft_dist = msg.ranges[i]

        self.min_backleft_dist = msg.ranges[l]
        for i in back_left:
            if msg.ranges[i] < self.min_backleft_dist:
                self.min_backleft_dist = msg.ranges[i]

        self.min_backright_dist = msg.ranges[r]
        for i in back_right:
            if msg.ranges[i] < self.min_backright_dist:
                self.min_backright_dist = msg.ranges[i]

        self.min_frontright_dist = msg.ranges[fr]
        for i in front_right:
            if msg.ranges[i] < self.min_frontright_dist:
                self.min_frontright_dist = msg.ranges[i]

        # NOTE Eric's deprecated code 

        # self.min_front_dist = msg.ranges[0]
        # for i in range(-5, 5):
        #     if msg.ranges[i] < self.min_front_dist:
        #         self.min_front_dist = msg.ranges[i]

        # # min_frontleft_dist = msg.ranges[95]
        # # for i in range(48,142):
        # #     if msg.ranges[i] < min_frontleft_dist:
        # #         min_frontleft_dist = msg.ranges[i]

        # # min_left_dist = msg.ranges[190]
        # # for i in range(143,237):
        # #     if msg.ranges[i] < min_left_dist:
        # #         min_left_dist = msg.ranges[i]

        # self.min_right_dist = msg.ranges[90]
        # for i in range(68, 112):
        #     if msg.ranges[i] < self.min_right_dist:
        #         self.min_right_dist = msg.ranges[i]

        # self.min_frontright_dist = msg.ranges[45]
        # for i in range(23, 67):
        #     if msg.ranges[i] < self.min_frontright_dist:
        #         self.min_frontright_dist = msg.ranges[i]
        
        # NOTE Direction Booleans deprecated
        # if min_frontright_dist < 0.2 and min_front_dist > 0.2:
        #     self.turnLeft = True
        #     self.turnRight = False
        #     self.get_logger().info("Left")
        # elif min_frontright_dist > 0.2 and min_frontright_dist < 0.4:
        #     self.turnLeft = False
        #     self.turnRight = True
        #     self.get_logger().info("Right")
        # else:
        #     self.turnLeft = False
        #     self.turnRight = False
        
        # if min_front_dist < 0.2:
        #     self.turnLeft = False
        #     self.turnRight = False
        #     if min_right_dist < 0.25:
        #         self.turnSharpLeft = True
        #         self.turnSharpRight = False
        #         self.get_logger().info("Sharp Left")
        #     else:
        #         self.turnSharpLeft = False
        #         self.turnSharpRight = True
        #         self.get_logger().info("Sharp Right1")
        # else:
        #     self.turnSharpLeft = False
        #     self.turnSharpRight = False

        # if min_frontright_dist > 0.3 and min_right_dist > 0.4:
        #     self.turnLeft = False
        #     self.turnRight = False
        #     self.turnFront = False
        #     self.turnSharpRight = True
        #     self.get_logger().info("Sharp Right2")
        # else:
        #     self.turnSharpRight = False


    def shutdown_cmd(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    my_follower = Follow()
    rclpy.spin(my_follower)
    my_follower.shutdown_cmd()
    my_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


