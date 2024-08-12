#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class ImuToBaseLink:
    def __init__(self):
        rospy.init_node('imu_to_base_link')
        
        self.imu_subscriber = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
        self.gps_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        self.heading_subscriber = rospy.Subscriber('/gps/navheading', Imu, self.heading_callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(10)
        
        # Initialize position variables
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.heading = 0.0  # Initial heading

        # Reference coordinates (in degrees)
        self.ref_lat = 56.1723059
        self.ref_lon = 10.191491
        self.ref_alt = 122.895  # Assuming the reference altitude is 0, adjust if needed
        
        # Offset in degrees for roll, pitch, and yaw
        self.offset_roll_degrees = 0
        self.offset_pitch_degrees = 180
        self.offset_yaw_degrees = 45

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

    def heading_callback(self, msg):
        # Extract heading from /gps/navheading message
        orientation_q = msg.orientation
        (_, _, self.heading) = tf.transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def geodetic_to_enu(self, lat, lon, alt, ref_lat, ref_lon, ref_alt):
        # WGS-84 ellipsoid parameters
        a = 6378137.0  # Earth's semi-major axis in meters
        f = 1 / 298.257223563  # Flattening
        e_sq = f * (2 - f)  # Square of eccentricity

        # Convert latitude and longitude from degrees to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        # Compute prime vertical radius of curvature
        N = a / math.sqrt(1 - e_sq * math.sin(ref_lat_rad) ** 2)

        # Compute ECEF coordinates of reference point
        ref_x = (N + ref_alt) * math.cos(ref_lat_rad) * math.cos(ref_lon_rad)
        ref_y = (N + ref_alt) * math.cos(ref_lat_rad) * math.sin(ref_lon_rad)
        ref_z = (N * (1 - e_sq) + ref_alt) * math.sin(ref_lat_rad)
        
        # Compute ECEF coordinates of current point
        N = a / math.sqrt(1 - e_sq * math.sin(lat_rad) ** 2)
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e_sq) + alt) * math.sin(lat_rad)

        # Compute the differences in ECEF coordinates
        dx = x - ref_x
        dy = y - ref_y
        dz = z - ref_z

        # Define the rotation matrix
        R = np.array([
            [-math.sin(ref_lon_rad), math.cos(ref_lon_rad), 0],
            [-math.sin(ref_lat_rad) * math.cos(ref_lon_rad), -math.sin(ref_lat_rad) * math.sin(ref_lon_rad), math.cos(ref_lat_rad)],
            [math.cos(ref_lat_rad) * math.cos(ref_lon_rad), math.cos(ref_lat_rad) * math.sin(ref_lon_rad), math.sin(ref_lat_rad)]
        ])

        # Compute local ENU coordinates
        enu = np.dot(R, np.array([dx, dy, dz]))
        return enu[0], enu[1], enu[2]

    def imu_callback(self, msg):
        try:
            # Ensure GPS data is available
            if self.latitude is None or self.longitude is None or self.altitude is None:
                return

            # Lookup the transformation from imu_tf to base_link
            (trans_ba, rot_ba) = self.listener.lookupTransform('base_link', 'imu_tf', rospy.Time(0))
            rot_ba_mat = tf.transformations.quaternion_matrix(rot_ba)

            # Lookup the transformation from gps_tf to base_link
            (trans_ca, rot_ca) = self.listener.lookupTransform('base_link', 'gps_tf', rospy.Time(0))
            trans_ca_vec = np.array([trans_ca[0], trans_ca[1], trans_ca[2]])

            # Convert IMU orientation from quaternion to rotation matrix
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])

            # Apply offsets to Euler angles
            offset_roll_rad = math.radians(self.offset_roll_degrees)
            offset_pitch_rad = math.radians(self.offset_pitch_degrees)
            offset_yaw_rad = math.radians(self.offset_yaw_degrees)
 
            roll += offset_roll_rad
            pitch += offset_pitch_rad
            yaw += offset_yaw_rad
            # Convert Euler angles back to quaternion
            offset_quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            # Convert quaternion to rotation matrix
            imu_rot_mat = tf.transformations.quaternion_matrix(offset_quaternion)

            # Calculate the rotation from world frame to base_link
            world_rot_mat = np.dot(rot_ba_mat, imu_rot_mat)
            world_quat = tf.transformations.quaternion_from_matrix(world_rot_mat)

            (w_roll, w_pitch, w_yaw1) = tf.transformations.euler_from_quaternion(world_quat)

            gps_yaw = math.degrees(self.heading) - 90
            
            imu_yaw = math.radians(imu_yaw)
            w_yaw = math.radians(gps_yaw)
            # Convert Euler angles back to quaternion
            world_quat = tf.transformations.quaternion_from_euler(w_roll, w_pitch, w_yaw)
            world_rot_mat = imu_rot_mat = tf.transformations.quaternion_matrix(world_quat)
            # Convert GPS coordinates to local ENU coordinates
            x, y, z = self.geodetic_to_enu(self.latitude, self.longitude, self.altitude, self.ref_lat, self.ref_lon, self.ref_alt)

            # Transform the GPS position to the base_link frame
            gps_pos_world = np.array([x,y,z]) - np.dot(world_rot_mat[:3, :3], trans_ca_vec)
            #gps_pos_world = -np.dot(world_rot_mat[:3, :3], trans_ca_vec)
            # Create the TransformStamped message
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "base_link"

            # Set translation from GPS data
            t.transform.translation.x = gps_pos_world[0]
            t.transform.translation.y = gps_pos_world[1]
            t.transform.translation.z = gps_pos_world[2]

            # Set rotation using existing pitch, roll, and updated heading
            t.transform.rotation.x = world_quat[0]
            t.transform.rotation.y = world_quat[1]
            t.transform.rotation.z = world_quat[2]
            t.transform.rotation.w = world_quat[3]

            # Broadcast the transformation
            self.br.sendTransform(t)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ImuToBaseLink()
    node.run()
