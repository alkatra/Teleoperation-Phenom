import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import serial
port = '/dev/ttyACM0'
import time
import threading

class ScaredRobot(Node):

    def controller_command_callback(self, msg):
        try:
            if msg.axes[1] > 0.2:              # Axis: Movement Stick pushed forward. Request Forward.
                self.direction_request = "F"    
            elif msg.axes[1] < -0.2:            # Axis: Movement Stick pushed backward. Request Backward.
                self.direction_request = "B"
            elif msg.axes[0] > 0.2:             # Axis: Movement Stick turned left. Request Turn.
                self.direction_request = "L"
            elif msg.axes[0] < -0.2:            # Axis: Movement Stick turned right. Request Turn.
                self.direction_request = "R"
            else:                               # No Turn Requested.       
                self.direction_request = "S"    
            self.get_logger().info("RECEIVED: " + self.direction_request)


        except:
            self.get_logger().warn("[SKIPPING] Error while parsing controller command")

    def sensor_data_callback(self, msg):
        self.get_logger().info("RECEIVED: " + msg.data)

        try:
            clean = msg.data[1:]                                      
            clean2 = clean[:-2]                                 
            readings = clean2.split(',')

            self.left_last = self.left = int((readings[0])[:-2])
            self.front_last = self.front = int((readings[1])[:-2])
            self.right_last = self.right = int((readings[2])[:-2])
        except:
            self.get_logger().warn("[SKIPPING] Error while parsing sensor data")
        self.publish_command()

    def publish_command(self):

        # Moving back takes highest precedence:
        if(self.direction_request == "B"):
            self.next_msg = "MOVEB\n"
        # Interrupt commands depending on sensor readings:
        elif(self.front < 20 or self.left < 20 or self.right < 20):
            if(self.left >= self.right and self.last_msg != "PIVL\n"):
                self.next_msg = "PIVL\n" # Pivot left to avoid obstacle
            elif(self.right > self.left and self.last_msg != "PIVR\n"): 
                self.next_msg = "PIVR\n" # Pivot right to avoid obstacle

        else:   # No interrupt commands. Send command from controller.
            if(self.direction_request == "F"):
                self.next_msg = "MOVEF\n"
            elif(self.direction_request == "L"):
                self.next_msg = "TURNL\n"
            elif(self.direction_request == "R"):
                self.next_msg = "TURNR\n"
            else:
                self.next_msg = "STOPN\n"


    def __init__(self):
        super().__init__('Controller')
        self.direction_request = "S"
        self.last_msg = self.next_msg = "STOPN\n"
        self.publish_thread = threading.Thread(target=self._publish_thread)
        self.publish_thread.daemon = True
        self.publish_thread.start()

        # Subscribe to controller commands
        self.subscriptionJoy = self.create_subscription(
            Joy,
            '/joy',
            self.controller_command_callback,
            1)
        self.subscriptionJoy  

        # Subscribe to sensor data from robot's bridge
        self.subscriptionSensor = self.create_subscription(
            String,
            '/robot/all',
            self.sensor_data_callback,
            1)
        self.subscriptionSensor  



    # Publish message only if it is different to last message
    def _publish_thread(self):
        ser = serial.Serial(port)
        try:
            while True:
                if self.last_msg != self.next_msg:
                    self.get_logger().info("Publishing: "+self.next_msg)
                    ser.write(self.next_msg.encode())
                    self.last_msg = self.next_msg
                # time.sleep(0.05)
        except:
            ser.close()
      

def main(args=None):
    rclpy.init(args=args)

    scaredrobot = ScaredRobot()
    rclpy.spin(scaredrobot)
    
    scaredrobot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
