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
                    #  Először lekérem a servo motorok státuszát az Arduinotól serial buson...
                    bla="00"
                    if arduino.in_waiting==0:
                            for i in bla:
                                arduino.write(bytes(i, "UTF-8"))
                                time.sleep(0.2)
                            print("státuszt kérek")
                    time.sleep(0.2)
                    # státusz válasz feldolgozása - servo motor aktuális szögek változóba elrakva
                    status = "S1P090S2P090S3P110S4P025"
                    servo_1 = int(status[3:6])     # alap motor (forgás)
                    servo_2 = int(status[9:12])    # Joint1 (előre-hátra)
                    servo_3 = int(status[15:18])   # Joint2 (fel-le)
                    servo_4 = int(status[22:])     # Megfogó
                    # ezután feldolgozom a Twist üzenetben kapott mozgásinfót...
                    if msg.linear.z != 0:
                        servo_1 += msg.linear.z
                    if msg.linear.x != 0:
                        servo_1 += msg.linear.x
                    if msg.linear.y != 0:
                        servo_1 += msg.linear.y
                    # kell még a megfogó - melyik gomb legyen?????
                    Servo_4= 25

                    # összerakom az üzenetet
                    bla= "S1P" + Str(servo_1) + "S2P" + Str(servo_2) + "S3P" + Str(servo_3) + "S4P" + Str(servo_4)
                    if arduino.in_waiting==0:
                        for i in bla:
                            arduino.write(bytes(i, "UTF-8"))
                            time.sleep(0.2)
                            print("mozgás!")
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
