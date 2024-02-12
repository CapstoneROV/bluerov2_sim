#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Wrench
from gazebo_msgs.srv import ApplyBodyWrench
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.listener import TransformListener
from tf import TransformBroadcaster

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class GazeboWrenchController(object):
    def __init__(self, kp=10, ki=0, kd=0):
        rospy.init_node('bluerov_sitl', anonymous=True, log_level=rospy.DEBUG);
        rospy.loginfo("SITL PID STARTED");
        self.pose_subscriber = rospy.Subscriber('/bluerov2/mavros/local_position/pose', PoseStamped, self.pose_callback, queue_size=1)
        self.apply_wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.body_name = 'bluerov2::bluerov2/base_link'  # Adjust based on your model and link names in Gazebo

        self.force_pid = PIDController(kp=kp, ki=ki, kd=kd)
        self.torque_pid = PIDController(kp=kp, ki=ki, kd=kd)

        self.last_time = rospy.Time.now()

    def pose_callback(self, msg):
        try:
            # Wait for the transform to be available
            self.tf_listener.waitForTransform('world', 'bluerov2/base_link', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('world', 'bluerov2/base_link', rospy.Time(0))
            rospy.loginfo("Baselink transform: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(trans[0], trans[1], trans[2]))

            mavros_pose = msg.pose
            # Log mavros pos 
            rospy.loginfo("Mavros Local Position: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(mavros_pose.position.x, mavros_pose.position.y, mavros_pose.position.z))

            tf_pose = PoseStamped()
            tf_pose.pose.position.x, tf_pose.pose.position.y, tf_pose.pose.position.z = trans
            tf_pose.pose.orientation.x, tf_pose.pose.orientation.y, tf_pose.pose.orientation.z, tf_pose.pose.orientation.w = rot

            # Broadcase this tf_pose

            self.apply_control(mavros_pose, tf_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF lookup error: %s" % e)

    def apply_control(self, mavros_pose, tf_pose):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Calculate position error
        error_x = -tf_pose.pose.position.x + mavros_pose.position.x
        error_y = -tf_pose.pose.position.y + mavros_pose.position.y
        error_z = -tf_pose.pose.position.z + mavros_pose.position.z

        # Log errors
        rospy.loginfo("Position errors - x: {:.2f}, y: {:.2f}, z: {:.2f}".format(error_x, error_y, error_z))

        # Calculate force
        force_x = self.force_pid.update(error_x, dt)
        force_y = self.force_pid.update(error_y, dt)
        force_z = self.force_pid.update(error_z, dt)

        # Calculate orientation error
        mavros_orientation = [mavros_pose.orientation.x, mavros_pose.orientation.y, mavros_pose.orientation.z, mavros_pose.orientation.w]
        tf_orientation = [tf_pose.pose.orientation.x, tf_pose.pose.orientation.y, tf_pose.pose.orientation.z, tf_pose.pose.orientation.w]
        orientation_error = tf.transformations.quaternion_multiply(tf_orientation, tf.transformations.quaternion_inverse(mavros_orientation))

        # Simplified torque calculation
        torque_x = self.torque_pid.update(orientation_error[0], dt)
        torque_y = self.torque_pid.update(orientation_error[1], dt)
        torque_z = self.torque_pid.update(orientation_error[2], dt)

        # Apply wrench
        wrench = Wrench()
        wrench.force.x = force_x
        wrench.force.y = force_y
        wrench.force.z = force_z
        wrench.torque.x = torque_x
        wrench.torque.y = torque_y
        wrench.torque.z = torque_z
        
        try:
            self.apply_wrench_service(body_name=self.body_name, wrench=wrench, reference_frame='world', start_time=rospy.Time.now(), duration=rospy.Duration.from_sec(0.1))
        except rospy.ServiceException as e:
            rospy.logerr("Apply wrench service call failed: %s" % e)

if __name__ == '__main__':
    try:
        # Get parameters from ROS node
        kp = rospy.get_param('~kp', 10)
        ki = rospy.get_param('~ki', 0)
        kd = rospy.get_param('~kd', 0)

        # Create GazeboWrenchController object with parameters
        controller = GazeboWrenchController(kp, ki, kd)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        controller = GazeboWrenchController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
