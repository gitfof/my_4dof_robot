import rclpy
import serial, time
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Robot_Subscriber(Node):

    def __init__(self):
        super().__init__('Robot_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
            time.sleep(0.1);
            if arduino.isOpen():
                try:
                    while True:
                        if msg.linear.x>0:
                            arduino.write("bal");
                        elif msg.linear.x<0:
                            arduino.write("jobb");
                except:
                    print("hiba");  

def main(args=None):
    rclpy.init(args=args)

    robot = Robot_Subscriber()

    rclpy.spin(robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
