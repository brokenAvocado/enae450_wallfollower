"""
    Minimal code for wall follower 
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
        self.dir = 0
        self.front_wall = False
        self.frontright_wall = False
        self.frontleft_wall = False

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
        #     msg.linear.x = 1.0  
        # elif key == 'b':
        #     msg.linear.x = -1.0  
        # else:
        #     msg.linear.x = 0.0



        # if (self.dir != 0):
        #     if self.dir-180 >= 0:
        #         msg.angular.z = -1.0
        #     elif self.dir-180 < 0:
        #         msg.angular.z = 1.0
        # else:
        #     msg.angular.z = 0.0

        if self.front_wall == False:
            if (self.frontright_wall == True) & (self.frontleft_wall == False):
                msg.linear.x = 0.08
                msg.angular.z = 1.0
            elif (self.frontright_wall == False) & (self.frontleft_wall == True):
                msg.linear.x = 0.08
                msg.angular.z = -1.0
            else:
                msg.linear.x = 0.08
                msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 1.0


        self.publisher_.publish(msg)
        
        
  
    
    def listener_callback(self,msg):
        '''
        Subscription Callback 
        TODO: implement
        '''

        min_front_dist = msg.ranges[0]
        for i in range(-22,23):
            if msg.ranges[i] < min_front_dist:
                min_front_dist = msg.ranges[i]

        min_frontleft_dist = msg.ranges[45]
        for i in range(23,68):
            if msg.ranges[i] < min_frontleft_dist:
                min_frontleft_dist = msg.ranges[i]

        min_left_dist = msg.ranges[90]
        for i in range(68,113):
            if msg.ranges[i] < min_left_dist:
                min_left_dist = msg.ranges[i]

        min_right_dist = msg.ranges[270]
        for i in range(247,293):
            if msg.ranges[i] < min_right_dist:
                min_right_dist = msg.ranges[i]

        min_frontright_dist = msg.ranges[315]
        for i in range(293,338):
            if msg.ranges[i] < min_frontright_dist:
                min_frontright_dist = msg.ranges[i]

        if min_front_dist < 0.4:
            self.front_wall = True
            self.forward_stop = True
        else:
            self.front_wall = False
            self.forward_stop = False

        if min_frontright_dist < 0.4:
            self.frontright_wall = True
        else:
            self.frontright_wall = False

        if min_frontleft_dist < 0.4:
            self.frontleft_wall = True
        else:
            self.frontleft_wall = False


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