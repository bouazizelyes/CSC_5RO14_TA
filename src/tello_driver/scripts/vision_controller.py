#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError

class VisionFlightController:
    def __init__(self):
        rospy.init_node('vision_flight_controller', anonymous=True)
        
        # Tools
        self.bridge = CvBridge()
        self.velocity_msg = Twist()
        self.is_flying = False
        
        # Publishers (Sending commands TO the drone)
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        
        # Subscribers (Getting data FROM the drone)
        # The callback function 'process_image' runs every time a new frame arrives
        self.image_sub = rospy.Subscriber('/tello/image_raw', Image, self.process_image, queue_size=1)
        
        rospy.sleep(1) # Wait for connections to establish

    def takeoff(self):
        rospy.loginfo("Taking off...")
        self.takeoff_pub.publish(Empty())
        rospy.sleep(5) # Wait to stabilize
        self.is_flying = True

    def land(self):
        rospy.loginfo("Landing...")
        # Stop all movement before landing
        self.cmd_vel_pub.publish(Twist()) 
        self.land_pub.publish(Empty())
        self.is_flying = False

    def process_image(self, data):
        """This is the main Flight Loop. It triggers on every new camera frame."""
        if not self.is_flying:
            return # Don't process vision for movement if we are on the ground

        try:
            # 1. Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # 2. RUN COMPUTER VISION ALGORITHM
        # ---------------------------------------------------------
        # Placeholder: Let's use simple Edge Detection to simulate finding a wall.
        # If there are too many edges (meaning we are close to a textured wall), back up!
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        
        # Calculate what percentage of the screen is "edges"
        edge_density = np.sum(edges == 255) / edges.size
        
        # 3. CALCULATE FLIGHT COMMANDS BASED ON CV
        # ---------------------------------------------------------
        # Reset velocities
        self.velocity_msg.linear.x = 0.0 # Forward/Back
        self.velocity_msg.linear.y = 0.0 # Left/Right
        self.velocity_msg.linear.z = 0.0 # Up/Down
        self.velocity_msg.angular.z = 0.0 # Rotate
        
        threshold = 0.05 # 5% of pixels are edges

        if edge_density > threshold:
            rospy.loginfo(f"Wall detected! Edge density: {edge_density:.2f}. Backing up!")
            self.velocity_msg.linear.x = -0.2  # Move backward
        else:
            rospy.loginfo(f"Path clear. Edge density: {edge_density:.2f}. Moving forward.")
            self.velocity_msg.linear.x = 0.2   # Move forward
            
        # 4. SEND COMMAND TO DRONE
        # ---------------------------------------------------------
        self.cmd_vel_pub.publish(self.velocity_msg)
        
        # Optional: Show the camera feed on your computer for debugging
        cv2.imshow("Tello Camera Feed", cv_image)
        cv2.imshow("Wall Detection (Edges)", edges)
        cv2.waitKey(3) # Required for OpenCV to update the window

if __name__ == '__main__':
    controller = VisionFlightController()
    
    try:
        controller.takeoff()
        # rospy.spin() keeps the script running, allowing the 'process_image' 
        # callback to continuously loop and fly the drone based on the camera.
        rospy.spin() 
        
    except rospy.ROSInterruptException:
        pass
    finally:
        # If you hit Ctrl+C, ensure the drone lands safely!
        controller.land()
        cv2.destroyAllWindows()

