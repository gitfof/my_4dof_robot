import rclpy
import serial, time
from rclpy.node import Node
from my_interfaces.msg import Robot

class Robot_Subscriber(Node):

    def __init__(self):
        super().__init__('Robot_subscriber')
        self.subscription = self.create_subscription(Robot,'/robot_controller',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.arduino = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=1)
        time.sleep(2)

    def listener_callback(self, msg):
        if self.arduino.isOpen():
            try:
                print("RPI_Node: waiting for any news...")
                if self.arduino.in_waiting > 0:
                    status = self.arduino.readline()
                    status.strip()
                    print(status)
                
                #  Ask for servo status from Arduino via Serial port...
                bla="00"
                print("RPI_Node: sending status request to Arduino...")
                if self.arduino.in_waiting==0:
                    for i in bla:
                        self.arduino.write(bytes(i, "UTF-8"))
                        #time.sleep(0.2)
                    print("státuszt kérek")
                
                while self.arduino.in_waiting==0: pass
                
                print("RPI_Node: waiting for status report from Arduino...")
                # Process status report - update joint angles
                if self.arduino.in_waiting > 0:
                    status = self.arduino.readline()
                    status.strip()
                    print(status)
                    servo_1 = int(status[1:3])     # basic joint (turn left-right)
                    servo_2 = int(status[4:6])    # shoulder (front-back)
                    servo_3 = int(status[7:9])   # ankle (up-down)
                    servo_4 = int(status[10:12])     # gripper

                    # Not used in 4DOF robot...
                    servo_5 = int(status[13:15])
                    servo_6 = int(status[16:18])
                    
                    # Process ROS Robot type message (got from the ROS server node)...
                    if msg.joint1 != 0:
                        servo_1 += int(msg.joint1)
                        if (servo_1 > 180): servo_1 = 180
                        elif (servo_1 < 0): servo_1 = 0
                    if msg.joint2 != 0:
                        servo_2 += int(msg.joint2)
                        if (servo_2 > 140): servo_2 = 140
                        elif (servo_2 < 80): servo_2 = 80
                    if msg.joint3 != 0:
                        servo_3 += int(msg.joint3)
                        if (servo_3 > 140) : servo_3 = 140
                        elif (servo_3 < 80) : servo_3 = 80
                    if msg.joint4 != 0:
                        servo_4 += int(msg.joint4)
                        if (servo_4 > 140) : servo_4 = 140
                        elif (servo_4 < 25) : servo_4 = 25
                    
                    # Not used in 4DOF robot...
                    if msg.joint5 != 0:
                        servo_5 += int(msg.joint5)
                        if (servo_5 > 180) : servo_5 = 180
                        elif (servo_5 < 0) : servo_5 = 0
                    if msg.joint6 != 0:
                        servo_6 += int(msg.joint6)
                        if (servo_6 > 180) : servo_6 = 180
                        elif (servo_6 < 0) : servo_6 = 0                    

                    print("RPI_Node: Creating move command for Arduino...")
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

                    print("RPI_Node: Send move command to Arduino...")
                    bla= "S1P" + szog_1 + "S2P" + szog_2 + "S3P" + szog_3 + "S4P" + szog_4
                    print(bla)
                    if self.arduino.in_waiting==0:
                        for i in bla:
                            self.arduino.write(bytes(i, "UTF-8"))
                            #time.sleep(0.2)
                        print("Move!")
                    while self.arduino.in_waiting==0: pass
                    # Check result of the movement
                    print("RPI_Node: waiting for response from Arduino about finishing the movement...")
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

