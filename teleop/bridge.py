import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
import serial
import threading
from rclpy.clock import Clock

port = '/dev/ttyACM0'

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            try:
                data = self.s.read(i)
            except:
                data = self.old_data
            self.old_data = data
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


class Robot(Node):
    def listener_callback(self, msg):
        print("exciting")
        command = msg.data 
        command +="\n" 
        self.get_logger().info('Sending data to serial port: "%s"' % msg.data)
        ser = serial.Serial(port)  
        print(ser.name)         
        ser.write(command.encode())
        ser.close()             

    
    def __init__(self):
        super().__init__('robot_bridge')

        self.subscription = self.create_subscription(
            String,
            '/robot/control',
            self.listener_callback,
            1)
        self.subscription  

        self.publish_thread = threading.Thread(target=self._publish_thread)
        self.publish_thread.daemon = True
        self.publish_thread.start()


    def _publish_thread(self):
        self.publisher_all = self.create_publisher(String, '/robot/all', 1)
        self.ultrasonic_left = self.create_publisher(Range, '/robot/left', 1)
        self.ultrasonic_center = self.create_publisher(Range, '/robot/center', 1)
        self.ultrasonic_right = self.create_publisher(Range, '/robot/right', 1)


        
        ser = serial.Serial(
            port=port,\
            baudrate=9600,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=0)

        self.ser = ser

        rl = ReadLine(ser)

        while(True):
            try:
                line = rl.readline().decode()
            except:
                line = self.old_line
            self.old_line = line

            if(len(line)==17):
                msg = String()

                msg.data = line
                self.publisher_all.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)

                try:
                    data = Range()
                    clean = line[1:]                                      
                    clean2 = clean[:-2]                                 
                    readings = clean2.split(',')
                    data.header.stamp = self.get_clock().now().to_msg()
                    data.header.frame_id = "ultrasonic_left"
                    data.radiation_type = 0
                    data.field_of_view = 0.52
                    data.min_range = 0.0
                    data.max_range = 99.9
                    data.range = float((readings[0])[:-2])
                    self.ultrasonic_left.publish(data)
                    data.header.frame_id = "ultrasonic_center"
                    data.range = float((readings[1])[:-2])
                    self.ultrasonic_center.publish(data)
                    data.header.frame_id = "ultrasonic_right"
                    data.range = float((readings[2])[:-2])
                    self.ultrasonic_right.publish(data)
                except Exception as e:
                    self.get_logger().warn(e)


            else:
                self.get_logger().info('SERIAL: "%s"' % line)

        

def main(args=None):
    rclpy.init(args=args)

    robot = Robot()
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
