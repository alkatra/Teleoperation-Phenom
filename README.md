# Teleoperation-Phenom

This repository is a comprehensive solution for the teleoperation of a Zumo Shield bot integrated with Arduino and three ultrasonic sensors. The project harnesses the capabilities of the Robot Operating System (ROS2 Foxy) for data communication, processing, and visualization, along with a web-based GUI for real-time bot control and sensor data monitoring.

Key Components:
1. **Arduino Code (arduino.ino)**: An Arduino sketch that interfaces with the ultrasonic sensors (left, right, and center) attached to the Zumo Shield bot. It reads the sensor data and transmits it via serial communication.
2. **Bridge Node**: A ROS node that reads the serial data from Arduino and publishes it to a ROS Topic named `/ulstrasonic/all`.
3. **Controller Node**: A ROS node that interfaces with a PS4 controller (through the ROS Joy Node) and the ultrasonic sensor data published on `/ultrasonic/all`. It uses this data to control the Zumo bot, sending commands back via Serial.
4. **Transform Nodes and URDF**: Includes three transformation nodes and a `robot_description.urdf` file to assist with data visualization in Rviz in the form of Range data.
5. **Frontend Folder (Web-based GUI)**: A web application built using ROSLib that communicates with the Bridge node. The GUI allows for Zumo robot control via graphical arrow keys, speed adjustment, and live display of sensor readings in a bar graph format.

The project provides a holistic teleoperation system from the physical robot with sensor integration, data processing in ROS, visualization in Rviz, to intuitive web-based user control interface.

*Note: You must have ROS and Arduino installed to use this project. Check the "Installation" section of the documentation for more information on setting up and running the project.*
