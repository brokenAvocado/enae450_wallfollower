"""
    Group 6: Wall Follower Code for Maze Solver
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
        
        # self.front_avg = float('inf')
        
        self.subscription= self.create_subscription(
            LaserScan,
            '/scan',  ## Read
            self.listener_callback,
            qos_profile,
        )
        timer_period = 0.5  # seconds

        # self.i = 0
        # self.dir = 0
        # self.front_wall = False
        # self.frontright_wall = False
        # self.backright = False
        # self.frontleft_wall = False

        self.turnLeft = False
        self.turnRight = False
        self.turnSharpLeft = False
        self.turnSharpRight = False

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
        # if key == 'm':
        #     msg.linear.x = 0.0
        #     msg.angular.z = 0.0
        #     return
        # if self.front_wall == False:
        #     if (self.frontright_wall == True) & (self.frontleft_wall == False):
        #         msg.linear.x = -0.08
        #         msg.angular.z = 0.5
        #     elif (self.frontright_wall == False) & (self.frontleft_wall == True):
        #         msg.linear.x = -0.08
        #         msg.angular.z = -0.5
        #     else:
        #         msg.linear.x = -0.08
        #         msg.angular.z = 0.0
        # else:
        #     msg.linear.x = 0.0
        #     msg.angular.z = 1.0
        if self.turnLeft:
            msg.linear.x = 0.08
            msg.angular.z = 0.5
        elif self.turnRight: 
            msg.linear.x = 0.08
            msg.angular.z = -0.5
        elif self.turnSharpLeft:
            msg.linear.x = 0.0
            msg.angular.z = 1.5
        elif self.turnSharpRight:
            msg.linear.x = 0.0
            msg.angular.z = -1.0
        else:
            msg.linear.x = 0.08
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        
        
  
    
    def listener_callback(self,msg):
        '''
        Subscription Callback 
        TODO: implement
        '''

        ########### Gazebo vs Real World #################
        # comment out the one not in use

        # Real World
        # front = range(-47,47)
        # right = range(522, 617)
        # front_right = range(618,712)
        # front_left = range(48,142)
        # left = range(143,237)
        # f = 0
        # fl = 95
        # l = 190
        # r = 570
        # fr = 665
        # bound1 = 0.2
        # bound2 = 0.4
        # bound3 = 0.25
        # bound4 = 0.3


        # Gazebo
        front = range(-7,7)
        right = range(247, 292)
        front_right = range(292,337)
        front_left = range(23,67)
        left = range(68,112)
        f = 0
        fl = 45
        l = 90
        r = 270
        fr = 315
        bound1 = 0.35
        bound2 = 0.28
        bound3 = 0.5
        bound4 = 0.15

        min_front_dist = msg.ranges[f]
        for i in front:
            if msg.ranges[i] < min_front_dist:
                min_front_dist = msg.ranges[i]

        # min_frontleft_dist = msg.ranges[fl]
        # for i in front_left:
        #     if msg.ranges[i] < min_frontleft_dist:
        #         min_frontleft_dist = msg.ranges[i]

        # min_left_dist = msg.ranges[190]
        # for i in left:
        #     if msg.ranges[i] < min_left_dist:
        #         min_left_dist = msg.ranges[i]

        min_right_dist = msg.ranges[r]
        for i in right:
            if msg.ranges[i] < min_right_dist:
                min_right_dist = msg.ranges[i]

        min_frontright_dist = msg.ranges[fr]
        for i in front_right:
            if msg.ranges[i] < min_frontright_dist:
                min_frontright_dist = msg.ranges[i]

        # if min_front_dist < bound2:
        #     self.front_wall = True
        #     self.forward_stop = True
        # else:
        #     self.front_wall = False
        #     self.forward_stop = False

        if min_frontright_dist < bound1 and min_front_dist > bound1:
            self.turnLeft = True
            self.turnRight = False
            self.get_logger().info("Left")
        elif min_frontright_dist > bound1 and min_front_dist > bound1:
            self.turnLeft = False
            self.turnRight = True
            self.get_logger().info("Right")
        else:
            self.turnLeft = False
            self.turnRight = False
        
        if min_front_dist < bound2:
            self.turnLeft = False
            self.turnRight = False
            if min_right_dist < bound3:
                self.turnSharpLeft = True
                self.turnSharpRight = False
                self.get_logger().info("Sharp Left")
            else:
                self.turnSharpLeft = False
                self.turnSharpRight = True
                self.get_logger().info("Sharp Right1")
        else:
            self.turnSharpLeft = False
            self.turnSharpRight = False

        # if min_frontright_dist > bound4 and min_right_dist > bound2:
        #     self.turnLeft = False
        #     self.turnRight = False
        #     self.turnFront = False
        #     self.turnSharpRight = True
        #     self.get_logger().info("Sharp Right2")
        # else:
        #     self.turnSharpRight = False


        # if min_frontleft_dist < bound2:
        #     self.frontleft_wall = True
        # else:
        #     self.frontleft_wall = False


        # self.front_dist = msg.ranges[0]
        # self.left_dist = msg.ranges[90]
        # self.right_dist = msg.ranges[270]

        # if self.front_dist < 0.50:
        #     self.get_logger().info('Wall Front')
        
        # if self.left_dist < 0.50:
        #     self.get_logger().info('Wall Left')
        
        # if self.right_dist < 0.50:
        #     self.get_logger().info('Wall Right')

        # if(msg.ranges[0] < 1):
        #     self.dir = 0
        # elif(msg.ranges[190] < 1):
        #     self.dir = 90
        # elif(msg.ranges[380] < 1):
        #     self.dir =  180
        # elif(msg.ranges[570] < 1):
        #     self.dir = 270
        # self.get_logger().info("Angle where Box is : %f" %(self.dir))

        # self.get_logger().info(str(len(msg.ranges)))
        # for i in range(337,360):
        #     if type(msg.ranges[i]) != 'inf':
        #         self.get_logger().info('no')
            # else:
        #         self.get_logger().info('yes')
        #     self.get_logger().info(str(msg.ranges[i]))

        # self.get_logger().info('I heard : Range[0] "%f" Ranges[90]: "%f"' %(msg.ranges[0] ,msg.ranges[90]))
        

def main(args=None):
    rclpy.init(args=args)
    my_follower = Follow()
    rclpy.spin(my_follower)
    my_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


