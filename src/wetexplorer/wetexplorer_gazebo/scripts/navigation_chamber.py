#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from itertools import permutations
import math
import tf

class NavGoals:
    def __init__(self):
        rospy.init_node('nav_goals', anonymous=True)

        # Initialize action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Offset from base_link to chamber_link
        self.chamber_offset = (0.48712, 0.0, -0.014912)

        # Original goals
        original_goals = [
            (1.5, 4.0),
            (4.0, 3.0),
            (5.0, 0.0),
            (4.0, -2.0),
            (1.0, -3.0),
            (1.0, 1.0),
            (-1.5, -5.0),
            (-4.5, -3.0),
            (-1.5, 0.0),
            (-3.5, 1.5)
        ]

        # Starting position offset
        self.start_position = (-6.5, 0.0)

        # Apply offset to goals
        self.goals = [(x - self.start_position[0], y - self.start_position[1]) for x, y in original_goals]

        # Find the optimal path
        self.goals = list(self.find_optimal_path(self.goals))

        # Compute orientations for the goals
        self.orientations = self.compute_orientations(self.goals)

        # Transform positions for chamber_link
        self.goals = [self.transform_to_chamber_link(x, y, orientation) for (x, y), orientation in zip(self.goals, self.orientations)]

        # Add the starting position as the last goal to return to the start
        self.goals.append(self.transform_to_chamber_link(self.start_position[0], self.start_position[1], self.orientations[-1]))

        # Start the process
        self.run()

    def transform_to_chamber_link(self, x, y, orientation):
        # Calculate the transformed position
        rotation = tf.transformations.euler_from_quaternion(orientation)[2]  # Extract yaw (rotation around Z-axis)
        
        x_chamber = x - self.chamber_offset[0] * math.cos(rotation) + self.chamber_offset[1] * math.sin(rotation)
        y_chamber = y - self.chamber_offset[0] * math.sin(rotation) - self.chamber_offset[1] * math.cos(rotation)
        
        return (x_chamber, y_chamber)

    def send_goal(self, x, y, orientation):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        rospy.loginfo(f"Sending goal: ({x}, {y}) with orientation: {orientation}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        state = self.client.get_state()
        
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal reached: ({x}, {y})")
            return True
        else:
            rospy.logwarn(f"Failed to reach goal: ({x}, {y})")
            return False

    def run(self):
        for idx in range(len(self.goals)):
            x, y = self.goals[idx]
            orientation = self.orientations[idx]

            if self.send_goal(x, y, orientation):
                rospy.loginfo(f"Reached goal {idx+1}: ({x}, {y}), waiting for 10 seconds...")
                rospy.sleep(10)
            else:
                rospy.logwarn(f"Skipping goal {idx+1}: ({x}, {y}) due to failure.")

        rospy.loginfo("All goals processed.")

    def calculate_orientation(self, x1, y1, x2, y2):
        # Calculate the angle between two points
        angle = math.atan2(y2 - y1, x2 - x1)

        # Convert the angle to a quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        return quaternion

    def compute_orientations(self, goals):
        orientations = []
        for i in range(len(goals) - 1):
            x, y = goals[i]
            next_x, next_y = goals[i + 1]
            orientation = self.calculate_orientation(x, y, next_x, next_y)
            orientations.append(orientation)

        # For the last goal, use the orientation to return to the start
        orientations.append(self.calculate_orientation(goals[-1][0], goals[-1][1], self.start_position[0], self.start_position[1]))
        return orientations

    def calculate_total_distance(self, path):
        total_distance = 0
        prev = self.start_position
        for point in path:
            total_distance += math.sqrt((point[0] - prev[0]) ** 2 + (point[1] - prev[1]) ** 2)
            prev = point
        # Return to start
        total_distance += math.sqrt((self.start_position[0] - prev[0]) ** 2 + (self.start_position[1] - prev[1]) ** 2)
        return total_distance

    def find_optimal_path(self, goals):
        # Generate all possible permutations of the goals
        permutations_of_goals = list(permutations(goals))
        
        # Find the permutation with the shortest total distance
        min_distance = float('inf')
        best_path = []
        
        for perm in permutations_of_goals:
            distance = self.calculate_total_distance(perm)
            if distance < min_distance:
                min_distance = distance
                best_path = perm
                
        return best_path

if __name__ == '__main__':
    try:
        NavGoals()
    except rospy.ROSInterruptException:
        pass
