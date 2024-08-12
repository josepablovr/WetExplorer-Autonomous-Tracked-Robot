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
        
   
        self.gps_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        self.heading_subscriber = rospy.Subscriber('/gps/navheading', Imu, self.heading_callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(10)
        
        # Initialize position variables
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.heading = None  # Initial heading

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
        self.geodetic_to_enu(self.latitude, self.longitude, self.altitude, self.ref_lat, self.ref_lon, self.ref_alt)

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
        x,y,z = enu[0], enu[1], enu[2]

        try:
            # Ensure GPS data is available
            if self.latitude is None or self.longitude is None or self.altitude is None or self.heading is None:
                return


            #INPUT
            
            # Lookup the transformation from gps_tf to base_link
            (t_gps_base, R_gps_base) = self.listener.lookupTransform('base_link', 'gps_tf', rospy.Time(0))
            
            # Position of gps frame in world coordinates
            P_gps_world = np.array([x,y,z])

            
            # Orientation of base frame in world coordinates
            base_yaw = math.degrees(self.heading) + 270
            w_yaw = math.radians(base_yaw)
            # Convert Euler angles back to quaternion
            R_base_world = tf.transformations.quaternion_from_euler(0, 0, w_yaw)

            R_base_world = tf.transformations.quaternion_matrix(R_base_world)[:3, :3]

            # Convert the quaternion to a rotation matrix
            R_gps_base_mat = tf.transformations.quaternion_matrix(R_gps_base)[:3, :3]

            # Position of base frame in world coordinates (initially unknown)
            # Compute the position of the base frame in world coordinates
            P_base_world = P_gps_world - np.dot(R_base_world, t_gps_base)

            # Compose the orientation of the base frame in world coordinates
            R_gps_world = np.dot(R_base_world, R_gps_base_mat)

            # Convert the resulting rotation matrix back to quaternion
            world_quat = tf.transformations.quaternion_from_matrix(np.vstack((np.hstack((R_gps_world, [[0],[0],[0]])), [0,0,0,1])))

            # Create the TransformStamped message
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "base_link"

            # Set translation from GPS data
            t.transform.translation.x = P_base_world[0]
            t.transform.translation.y = P_base_world[1]
            t.transform.translation.z = P_base_world[2]

            # Set rotation from the combined quaternion
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
