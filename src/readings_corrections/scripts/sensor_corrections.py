#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
import math
import numpy as np
import tf.transformations as tf_trans

class SensorOffsetCorrector:
    def __init__(self):
        rospy.init_node('sensor_offset_corrector')
        self.publish_transform = False
        self.imu_subscriber = rospy.Subscriber('/imu/data_transformed', Imu, self.imu_callback)
        self.imu_sub = rospy.Subscriber('/imu/gps_heading_corrected', Imu, self.imu2_callback)
        if self.publish_transform:
            self.br = tf2_ros.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(10)

        self.imu_publisher = rospy.Publisher('/imu/data_filtered', Imu, queue_size=10)
        self.imu_publisher2 = rospy.Publisher('/imu/gravity', Imu, queue_size=10)
        # Set alpha values for the low-pass filter
        self.alpha_acc = 1
        self.alpha_ang_vel = 0.3
        self.prev_acc = None
        self.prev_ang_vel = None

        self.alpha_grav = 0.3
        self.prev_grav = None
        self.initial_yaw = None
        self.yaw_offset = 0
        self.last_yaw = None
        self.yaw_drift = 0
        self.yaw_output = 0

        self.sample_frequency = 250
        self.cuttoff_frequency = 50
        alpha = 1/(1+1/(2*math.pi*1/self.sample_frequency*self.cuttoff_frequency))

        
        self.max_acc = 15
        self.max_vel = 5.0
        self.max_acc_grav = 9.81 + self.max_acc

        self.min_acc = 0
        self.acceleration_calibration = False
        self.calibration_time = 0.1
        self.init_calib_time = None
        self.gravity_magnitude = None
        

    def calibrate_acceleration(self, msg):
        if self.init_calib_time is None:
            self.init_calib_time  = rospy.Time.now()
        else:
            current_time = rospy.Time.now()
            filtered_grav_x = self.low_pass_filter(msg.linear_acceleration.x, self.prev_grav[0] if self.prev_grav else None, 0.001)
            filtered_grav_y = self.low_pass_filter(msg.linear_acceleration.y, self.prev_grav[1] if self.prev_grav else None, 0.001)
            filtered_grav_z = self.low_pass_filter(msg.linear_acceleration.z, self.prev_grav[2] if self.prev_grav else None, 0.001)
            
            if current_time.to_sec() - self.init_calib_time.to_sec() >= self.calibration_time:
                self.gravity_magnitude = math.sqrt(pow(filtered_grav_x, 2) + pow(filtered_grav_y,2) + pow(filtered_grav_z,2))
                print("GRAVITY MAGNITUDE: ", self.gravity_magnitude)
                #self.gravity_magnitude = 0
                self.acceleration_calibration  = True         


    def saturate(self,value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value

    def imu2_callback(self, msg):
        orientation_q = msg.orientation
        _, _, yaw = tf_trans.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        if self.yaw_offset is None:            
            self.yaw_offset = yaw

        elif  self.last_yaw is not None:
            self.yaw_drift = yaw - self.last_yaw 

            #print(yaw, self.yaw_output)
        


    def low_pass_filter(self,new_value, prev_value, alpha):
        if prev_value is None:
            # If no previous value, return the new value as the filtered value
            return new_value
        
        return alpha * new_value + (1 - alpha) * prev_value

    def acc_rejection(self, acc):

        acc_corrected = 0
        if abs(acc) <= self.min_acc:
            acc_corrected = 0.0
        elif acc < 0.0:
            acc_corrected = acc + self.min_acc
        else:
            acc_corrected = acc - self.min_acc
        return acc_corrected
        
    def imu_callback(self, msg):
        # Correct roll by adding 180 degrees
        
        if self.yaw_offset is not None:

            if self.acceleration_calibration is False: 
                self.calibrate_acceleration(msg)
            else:

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w
                    ])
                roll_degrees = math.degrees(roll) + 180
                #print(roll_degrees)
                roll_rad = math.radians(roll_degrees)

                # Correct pitch by negating the pitch angle
                pitch_rad = -pitch  # Negate pitch

                # Keep yaw as it is
                

                yaw_rad = -yaw
            

                if self.initial_yaw is None: 
                    self.initial_yaw = yaw_rad


                yaw_rad = yaw_rad - self.initial_yaw + self.yaw_offset 

                self.last_yaw =yaw_rad

                yaw = yaw_rad + self.yaw_drift
                self.yaw_output = yaw
                # Convert corrected angles back to quaternion
                corrected_quaternion = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, self.yaw_output)
                # print(math.degrees(roll_rad), math.degrees(pitch_rad), math.degrees(yaw_rad))
            
                # Create a new IMU message
                corrected_imu = Imu()
                corrected_imu.header = msg.header
                corrected_imu.orientation.x = corrected_quaternion[0]
                corrected_imu.orientation.y = corrected_quaternion[1]
                corrected_imu.orientation.z = corrected_quaternion[2]
                corrected_imu.orientation.w = corrected_quaternion[3]

                # Copy over the original covariances
                corrected_imu.orientation_covariance = msg.orientation_covariance
                # Apply low-pass filter to linear acceleration
                



                filtered_grav_x = self.low_pass_filter(msg.linear_acceleration.x, self.prev_grav[0] if self.prev_grav else None, self.alpha_grav)
                filtered_grav_y = self.low_pass_filter(msg.linear_acceleration.y, self.prev_grav[1] if self.prev_grav else None, self.alpha_grav)
                filtered_grav_z = self.low_pass_filter(msg.linear_acceleration.z, self.prev_grav[2] if self.prev_grav else None, self.alpha_grav) 
                
                filtered_grav_x = self.saturate(filtered_grav_x, -self.max_acc_grav, self.max_acc_grav)
                filtered_grav_y = self.saturate(filtered_grav_y, -self.max_acc_grav, self.max_acc_grav)
                filtered_grav_z = self.saturate(filtered_grav_z, -self.max_acc_grav, self.max_acc_grav)


                
                
                #corrected_quaternion = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, 0)
            
                g_x, g_y, g_z = self.gravity_remover(roll_rad,pitch_rad)
            
                acc_x = filtered_grav_x - g_x
                acc_y = filtered_grav_y - g_y
                acc_z = filtered_grav_z - g_z

                
                

                filtered_acc_x = self.low_pass_filter(acc_x, self.prev_acc[0] if self.prev_acc else None, self.alpha_acc)
                filtered_acc_y = self.low_pass_filter(acc_y, self.prev_acc[1] if self.prev_acc else None, self.alpha_acc)
                filtered_acc_z = self.low_pass_filter(acc_z, self.prev_acc[2] if self.prev_acc else None, self.alpha_acc) 

                filtered_acc_x = self.saturate(filtered_acc_x, -self.max_acc, self.max_acc)
                filtered_acc_y = self.saturate(filtered_acc_y, -self.max_acc, self.max_acc)
                filtered_acc_z = self.saturate(filtered_acc_z, -self.max_acc, self.max_acc)
                
                filtered_acc_x = self.acc_rejection(filtered_acc_x)
                filtered_acc_y = self.acc_rejection(filtered_acc_y)
                filtered_acc_z = self.acc_rejection(filtered_acc_z)
                # Apply low-pass filter to angular velocity
                filtered_ang_vel_x = self.low_pass_filter(msg.angular_velocity.x, self.prev_ang_vel[0] if self.prev_ang_vel else None, self.alpha_ang_vel)
                filtered_ang_vel_y = self.low_pass_filter(msg.angular_velocity.y, self.prev_ang_vel[1] if self.prev_ang_vel else None, self.alpha_ang_vel)
                filtered_ang_vel_z = self.low_pass_filter(msg.angular_velocity.z, self.prev_ang_vel[2] if self.prev_ang_vel else None, self.alpha_ang_vel)
                
                filtered_ang_vel_x = self.saturate(filtered_ang_vel_x, -self.max_vel, self.max_vel)
                filtered_ang_vel_y = self.saturate(filtered_ang_vel_y, -self.max_vel, self.max_vel)
                filtered_ang_vel_z = self.saturate(filtered_ang_vel_z, -self.max_vel, self.max_vel)

                # Store the filtered values for the next iteration
                self.prev_acc = (filtered_acc_x, filtered_acc_y, filtered_acc_z)
                self.prev_ang_vel = (filtered_ang_vel_x, filtered_ang_vel_y, filtered_ang_vel_z)
                self.prev_grav = (filtered_grav_x, filtered_grav_y, filtered_grav_z)

                corrected_imu.angular_velocity.x = filtered_ang_vel_x
                corrected_imu.angular_velocity.y = filtered_ang_vel_y
                corrected_imu.angular_velocity.z = filtered_ang_vel_z

                # Set filtered linear acceleration
                corrected_imu.linear_acceleration.x = filtered_acc_x
                corrected_imu.linear_acceleration.y = filtered_acc_y
                corrected_imu.linear_acceleration.z = filtered_acc_z
            
                corrected_imu.angular_velocity_covariance = msg.angular_velocity_covariance
            
                corrected_imu.linear_acceleration_covariance = msg.linear_acceleration_covariance

                corrected_imu.orientation_covariance = [
                    0.0001, 0, 0,
                    0, 0.0001, 0,
                    0, 0, 0.0001
                ]

                # Modify the angular velocity covariance
                corrected_imu.angular_velocity_covariance = [
                    0.00001225, 0, 0,
                    0,  0.00001225, 0,
                    0, 0,  0.00001225
                ]

                

                #29.4 mm/s2
                corrected_imu.linear_acceleration_covariance = [
                    0.000864, 0, 0,
                    0, 0.000864, 0,
                    0, 0, 0.000864 
                ]


                # print(msg.linear_acceleration)
                self.imu_publisher.publish(corrected_imu)
                corrected_imu.linear_acceleration.x = g_x
                corrected_imu.linear_acceleration.y = g_y
                corrected_imu.linear_acceleration.z = g_z




                self.imu_publisher2.publish(corrected_imu)

            
                if self.publish_transform:
                    # Create the TransformStamped message
                    t = TransformStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "world"
                    t.child_frame_id = "base_link"

                    # Set translation from GPS data
                    t.transform.translation.x = 0
                    t.transform.translation.y = 0
                    t.transform.translation.z = 0

                    # Set rotation from the combined quaternion
                    t.transform.rotation.x = corrected_quaternion[0]
                    t.transform.rotation.y = corrected_quaternion[1]
                    t.transform.rotation.z = corrected_quaternion[2]
                    t.transform.rotation.w = corrected_quaternion[3]

                    # Broadcast the transformation
                    # self.br.sendTransform(t)

    def calculate_angles(self, filtered_acc_x, filtered_acc_y, filtered_acc_z):
        # Calculate roll
        roll = np.arctan2(filtered_acc_y, np.sqrt(filtered_acc_x**2 + filtered_acc_z**2))

        # Calculate pitch
        pitch = np.arctan2(-filtered_acc_x, np.sqrt(filtered_acc_y**2 + filtered_acc_z**2))

        # Convert to degrees
        roll_degrees = np.degrees(roll)
        pitch_degrees = np.degrees(pitch)

        return roll_degrees, pitch_degrees
    
    def angles_to_vector(self, roll, pitch, magnitude):
        # Convert angles from degrees to radians
        roll_rad = roll
        pitch_rad = pitch

        # Calculate the components of the vector
        acc_x = -magnitude * np.sin(pitch_rad)
        acc_y = magnitude * np.sin(roll_rad) * np.cos(pitch_rad)
        acc_z = magnitude * np.cos(roll_rad) * np.cos(pitch_rad)

        return acc_x, acc_y, acc_z
    def transform_vector(self,q, v):
        """
        Transforma un vector de la trama A a la trama B usando un cuaternión q.
        
        :param q: Cuaternión que representa la orientación de la trama A con respecto a la trama B.
        :param v: Vector en la trama A.
        :return: Vector en la trama B.
        """
        # Convertir el vector en una matriz de 4x1 (para incluir la componente de rotación homogénea)
        v = np.array(v)
        v_homogeneous = np.append(v, [0.0])

        # Calcular la rotación del vector usando el cuaternión
        q_conjugate = tf_trans.quaternion_conjugate(q)
        v_rotated = tf_trans.quaternion_multiply(
            tf_trans.quaternion_multiply(q, v_homogeneous),
            q_conjugate
        )

        # Retornar las tres primeras componentes del vector rotado
        return v_rotated[:3]

    def gravity_remover(self, roll, pitch):
        g_x, g_y, g_z = self.angles_to_vector(roll, pitch, self.gravity_magnitude)

        
        return g_x, g_y, g_z
       
        #return acc_x_no_gravity, acc_y_no_gravity, acc_z_no_gravity


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = SensorOffsetCorrector()
    node.run()
