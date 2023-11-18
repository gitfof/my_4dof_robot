import rclpy
import serial, time
from rclpy.node import Node
from my_interfaces.msg import Robot
from sensor_msgs.msg import JointState

class Robot_Subscriber(Node):

    def __init__(self):
        super().__init__('Robot_subscriber')
        self.subscription = self.create_subscription(Robot,'/robot_controller',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.arduino = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=1)
        self.publisher = self.create_publisher(JointState, 'farobot_jsp', 10)
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
                    servo_1 = int(status[0:3])     # basic joint (turn left-right)
                    servo_2 = int(status[3:6])    # shoulder (front-back)
                    servo_3 = int(status[6:9])   # ankle (up-down)
                    servo_4 = int(status[9:12])     # gripper    
                    # Not used in 4DOF robot...
                    servo_5 = int(status[12:15])
                    servo_6 = int(status[15:18])
                    
                    # Process ROS Robot type message (got from the ROS server node)...
                    if msg.joint1 >= 0:
                        servo_1 = int(msg.joint1)
                        if (servo_1 > 180): servo_1 = 180
                        elif (servo_1 < 0): servo_1 = 0
                    if msg.joint2 >= 0:
                        servo_2 = int(msg.joint2)
                        if (servo_2 > 100): servo_2 = 100
                        elif (servo_2 < 0): servo_2 = 0
                    if msg.joint3 >= 0:
                        servo_3 = int(msg.joint3)
                        if (servo_3 > 90) : servo_3 = 90
                        elif (servo_3 < 0) : servo_3 = 0
                    if msg.joint4 >= 0:
                        servo_4 = int(msg.joint4)
                        if (servo_4 > 90) : servo_4 = 90
                        elif (servo_4 < 0) : servo_4 = 0
                    
                    # Not used in 4DOF robot...
                    if msg.joint5 >= 0:
                        servo_5 = int(msg.joint5)
                        if (servo_5 > 180) : servo_5 = 180
                        elif (servo_5 < 0) : servo_5 = 0
                    if msg.joint6 >= 0:
                        servo_6 = int(msg.joint6)
                        if (servo_6 > 180) : servo_6 = 180
                        elif (servo_6 < 0) : servo_6 = 0                    

                    print("RPI_Node: Creating move command for Arduino...")
                    # creating the message
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
                        
                    szog_5 = str(int(servo_5))
                    if len(szog_5) == 1: 
                        szog_5 = "00" + szog_5
                    elif len(szog_5) == 2:
                        szog_5 = "0" + szog_5
                    szog_6 = str(int(servo_6))
                    if len(szog_6) == 1: 
                        szog_6 = "00" + szog_6
                    elif len(szog_6) == 2:
                        szog_6 = "0" + szog_6
                        

                    print("RPI_Node: Send move command to Arduino...")
                    bla= szog_1 + szog_2 + szog_3 + szog_4 + szog_5 + szog_6
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

                # After the movement publish the joint states
                jsm = JointState()
                jsm.header.stamp = self.get_clock().now().to_msg()
                jsm.name = ['base_to_base2','console_to_arm1','arm1_to_arm2','arm2_to_gripper']
                jsm.position = [float(servo_1), float(servo_2), float(servo_3), float(servo_4)]
                jsm.velocity = [0.0, 0.0, 0.0, 0.0]
                jsm.effort = [0.0, 0.0, 0.0, 0.0]
                self.publisher.publish(jsm)
            
            except:
                # TODO - hibakezelo agat ki kell majd dolgozni!!!
                
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

