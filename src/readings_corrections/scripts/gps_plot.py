#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading
import time

class GPSOdometryListener:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gps_odometry_listener', anonymous=True)

        # Lists to store x and y positions
        self.gps_positions = []
        self.filtered_positions = []

        # Subscribers to the two topics
        rospy.Subscriber("/odometry/gps", Odometry, self.gps_callback)
        rospy.Subscriber("/odometry/filtered_map", Odometry, self.filtered_callback)

        # Run for 50 seconds and then stop
        self.running = True
        threading.Thread(target=self.timer_thread).start()

    def gps_callback(self, data):
        if self.running:
            # Store GPS odometry data
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            self.gps_positions.append((x, y))

    def filtered_callback(self, data):
        if self.running:
            # Store filtered map odometry data
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            self.filtered_positions.append((x, y))

    def timer_thread(self):
        # Let the node run for 50 seconds
        time.sleep(47)
        self.running = False
        self.plot_positions()

    def plot_positions(self):
        # Plot the x vs y positions for both GPS and filtered map odometry
        gps_x, gps_y = zip(*self.gps_positions) if self.gps_positions else ([], [])
        filtered_x, filtered_y = zip(*self.filtered_positions) if self.filtered_positions else ([], [])

        plt.figure()
        #plt.scatter(gps_x, gps_y, c='r', label="GPS Odometry", marker='o',s=1)  # Plotting as red dots
        plt.plot(filtered_x, filtered_y, 'b-', label="Filtered Map Odometry")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("X vs Y Position")
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    try:
        listener = GPSOdometryListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
