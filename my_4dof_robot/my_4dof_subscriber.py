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
            time.sleep(2);
            if arduino.isOpen():
                try:
                    if msg.linear.x > 0:
                        bla = "bal"
                        if arduino.in_waiting==0:
                            for i in bla:
                                arduino.write(bytes(i, "UTF-8"))
                                time.sleep(0.2)
                            print("balra megyunk")
                    elif msg.linear.x<0:
                        bla = "jobb"
                        if arduino.in_waiting==0:
                            for i in bla:
                                arduino.write(bytes(i, "UTF-8"))
                                time.sleep(0.2)
                            print("jobbra megyunk")    
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
