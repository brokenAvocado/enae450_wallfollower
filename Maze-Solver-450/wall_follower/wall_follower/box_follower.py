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
        self.subscription= self.create_subscription(
            LaserScan,
            '/scan',  ## Read
            self.listener_callback,
            qos_profile,
        )
        timer_period = 0.5  # seconds
        self.i = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.dir = 0.0
        
        
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
        #     msg.angular.z = 0.0

        if (self.dir != 0):
            if self.dir-180 >= 0:
                msg.angular.z = -1.0
            elif self.dir-180 < 0:
                msg.angular.z = 1.0
        else:
            msg.angular.z = 0.0
        self.publisher_.publish(msg)
        
    def listener_callback(self,msg):
        '''
        Subscription Callback 
        TODO: implement
        '''
        if(msg.ranges[0] < 1):
            self.dir = 0
        elif(msg.ranges[190] < 1):
            self.dir = 90
        elif(msg.ranges[380] < 1):
            self.dir =  180
        elif(msg.ranges[570] < 1):
            self.dir = 270
        self.get_logger().info("Angle where Box is : %f" %(self.dir))

def main(args=None):
    rclpy.init(args=args)
    my_follower = Follow()
    rclpy.spin(my_follower)
    my_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()