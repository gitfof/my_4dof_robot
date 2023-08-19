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
        self.arduino = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=1)
        time.sleep(2)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        #with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
            #time.sleep(2);
        if self.arduino.isOpen():
            try:
                print("ha van valami hír...")
                if self.arduino.in_waiting > 0:
                    status = self.arduino.readline()
                    status.strip()
                    print(status)
                    #  Először lekérem a servo motorok státuszát az Arduinotól serial buson...
                bla="00"
                print("státusz kéréshez készülni...")
                if self.arduino.in_waiting==0:
                    for i in bla:
                        self.arduino.write(bytes(i, "UTF-8"))
                        time.sleep(0.2)
                        print("státuszt kérek")
                while self.arduino.in_waiting==0: pass
                    # státusz válasz feldolgozása - servo motor aktuális szögek változóba elrakva
                print("státuszkérésre választ várok...")
                if self.arduino.in_waiting > 0:
                    status = arduino.readline()
                    status.strip()
                    print(status)
                    servo_1 = int(status[3:6])     # alap motor (forgás)
                    servo_2 = int(status[9:12])    # Joint1 (előre-hátra)
                    servo_3 = int(status[15:18])   # Joint2 (fel-le)
                    servo_4 = int(status[22:])     # Megfogó
                        # ezután feldolgozom a Twist üzenetben kapott mozgásinfót...
                    if msg.angular.z != 0:
                            #servo_1 += int(msg.linear.z * 5)
                        servo_1 = 45
                    if msg.linear.x != 0:
                            #servo_2 += int(msg.linear.x * 10)
                        servo_2 = 120
                    if msg.linear.y != 0:
                            # servo_3 += int(msg.linear.y * 10)
                        servo_3 = 130
                        # kell még a megfogó - melyik gomb legyen?????
                    Servo_4= 25

                    print("mozgáshoz a parancs összeállítása")
                        # összerakom az üzenetet
                    szog_1 = str(int(servo_1))
                    if len(szog_1) == 1: 
                        szog_1 = "00" + szog_1
                    elif len(szog_1) == 2:
                        szog_1 = "0" + szog_1
                    szog_2 = str(int(servo_2))
                    if len(szog_2) == 1: 
                        szog_2 = "00" + szog_2
                    elif len(szog_2) == 2:
                        szog_2 = "0" + szog_2
                    szog_3 = str(int(servo_3))
                    if len(szog_3) == 1: 
                        szog_3 = "00" + szog_3
                    elif len(szog_3) == 2:
                        szog_3 = "0" + szog_3
                    szog_4 = str(int(servo_4))
                    if len(szog_4) == 1: 
                        szog_4 = "00" + szog_4
                    elif len(szog_4) == 2:
                        szog_4 = "0" + szog_4

                    print("mozgás parancs kiküldése")
                    bla= "S1P" + szog_1 + "S2P" + szog_2 + "S3P" + szog_3 + "S4P" + szog_4
                    print(bla)
                    if self.arduino.in_waiting==0:
                        for i in bla:
                            self.arduino.write(bytes(i, "UTF-8"))
                            time.sleep(0.2)
                        print("mozgás!")
                    while self.arduino.in_waiting==0: pass
                        # ellenőrzöm a futtatás eredményét
                    print("várom a visszajelzést a mozgás befejezéséről")
                    if self.arduino.in_waiting > 0:
                        status = self.arduino.readline()
                        status.strip()
                        print(status)
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
