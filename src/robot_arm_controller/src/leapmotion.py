import rospy
from geometry_msgs.msg import Twist
import leap
import time

class MyListener(leap.Listener):
    def __init__(self, pub):
        super().__init__()
        self.last_position = None
        self.last_time = time.time()
        self.pub = pub

        # Low-pass filter coefficient
        self.alpha_linear_pos = 0.05
        self.alpha_linear_vel = 0.05
        self.alpha_angular_pos = 0.05
        self.alpha_angular_vel = 0.5
        
        # Previous filtered velocities
        self.filtered_linear_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.filtered_linear_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.filtered_angular_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.filtered_angular_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def low_pass_filter_linear_pos(self, new_value, index):
        # Apply the low-pass filter
        self.filtered_linear_position[index] = (self.alpha_linear_pos * new_value + (1 - self.alpha_linear_pos) * self.filtered_linear_position[index])
        return self.filtered_linear_position[index]
    
    def low_pass_filter_linear_vel (self, new_value, index):
        # Apply the low-pass filter
        self.filtered_linear_velocity[index] = (self.alpha_linear_vel * new_value + (1 - self.alpha_linear_vel) * self.filtered_linear_velocity[index])
        return self.filtered_linear_velocity[index]
    
    def low_pass_filter_angular_pos(self, new_value, index):
        # Apply the low-pass filter
        self.filtered_angular_position[index] = (self.alpha_angular_pos * new_value + (1 - self.alpha_angular_pos) * self.filtered_angular_position[index])
        return self.filtered_angular_position[index]
    
    def low_pass_filter_angular_vel(self, new_value, index):
        # Apply the low-pass filter
        self.filtered_angular_velocity[index] = (self.alpha_angular_vel * new_value + (1 - self.alpha_angular_vel) * self.filtered_angular_velocity[index])
        return self.filtered_angular_velocity[index]
    
    def on_connection_event(self, event):
        rospy.loginfo("Connected")

    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()

        rospy.loginfo(f"Found device {info.serial}")

    def normalize_velocity_x(self, value, max=100.0, deadzone=0.5):
        if value >= max:
            value = max
        if value <= -max:
            value = -max
        value = value/max
        if abs(value) <= deadzone:
            value = 0
        return value
      
    def normalize_velocity_y(self, value, max=90.0, deadzone=0.5):
        if value >= max:
            value = max
        if value <= -max:
            value = -max
        value = value/max
        if abs(value) <= deadzone:
            value = 0
        return value
    
    def normalize_velocity_z(self, value, max=160.0, deadzone=0.5):

        if value >= max:
            value = max
        if value <= -max:
            value = -max
        value = value/max
        if abs(value) <= deadzone:
            value = 0
        return value
    
    def normalize_angular_velocity(self, value, max = 0.3, deadzone=0.4):

        if value >= max:
            value = max
        if value <= -max:
            value = -max
        value = value/max
        if abs(value) <= deadzone:
            value = 0
        return value

    def on_tracking_event(self, event):

        current_time = time.time()
        time_diff = current_time - self.last_time
        rospy.loginfo(f"Frame {event.tracking_frame_id} with {len(event.hands)} hands with time diff {time_diff}")
        # Prepare a Twist message with zero velocities by default
        twist_msg = Twist()
        
        if len(event.hands) > 0:
            for hand in event.hands:
                palm_pos = [-hand.palm.position.z, -hand.palm.position.x, hand.palm.position.y, hand.palm.orientation.z, -hand.palm.orientation.x, hand.palm.orientation.y]
                palm_pos[0] = self.low_pass_filter_linear_pos(palm_pos[0], 0)
                palm_pos[1] = self.low_pass_filter_linear_pos(palm_pos[1], 1)
                palm_pos[2] = self.low_pass_filter_linear_pos(palm_pos[2], 2)
                palm_pos[3] = self.low_pass_filter_angular_pos(palm_pos[3], 3)
                palm_pos[4] = self.low_pass_filter_angular_pos(palm_pos[4], 4)
                palm_pos[5] = self.low_pass_filter_angular_pos(palm_pos[5], 5)

                if self.last_position is not None and time_diff > 0:
                    velocity = [
                        (palm_pos[0] - self.last_position[0]) / time_diff,
                        (palm_pos[1] - self.last_position[1]) / time_diff,
                        (palm_pos[2] - self.last_position[2]) / time_diff,
                        (palm_pos[3] - self.last_position[3]) / time_diff,
                        (palm_pos[4] - self.last_position[4]) / time_diff,
                        (palm_pos[5] - self.last_position[5]) / time_diff,
                    ]

                    velocity[0] = self.normalize_velocity_x(velocity[0])
                    velocity[1] = self.normalize_velocity_y(velocity[1])
                    velocity[2] = self.normalize_velocity_z(velocity[2])

                    # Apply low-pass filter
                    twist_msg.linear.x = self.low_pass_filter_linear_vel(velocity[0],0)
                    twist_msg.linear.y = self.low_pass_filter_linear_vel(velocity[1],1)
                    twist_msg.linear.z = self.low_pass_filter_linear_vel(velocity[2],2)

                    twist_msg.angular.x = self.low_pass_filter_angular_vel(velocity[3],3)
                    twist_msg.angular.y = self.low_pass_filter_angular_vel(velocity[4],4)
                    twist_msg.angular.z = self.low_pass_filter_angular_vel(velocity[5],4)


                    # Publish the normalized twist message
                    self.pub.publish(twist_msg)
                self.last_position = [palm_pos[0], palm_pos[1], palm_pos[2], palm_pos[3], palm_pos[4], palm_pos[5]]
        else:
            # If no hands detected, publish zero velocities
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            twist_msg.linear.z = 0
            self.pub.publish(twist_msg)
            self.last_position = None

        self.last_time = current_time  # Update last_time after processing all hands


def main():
    rospy.init_node('leap_motion_listener', anonymous=True)
    pub = rospy.Publisher('/spacenav/twist/', Twist, queue_size=10)

    my_listener = MyListener(pub)

    connection = leap.Connection()
    connection.add_listener(my_listener)

    try:
        with connection.open():
            connection.set_tracking_mode(leap.TrackingMode.Desktop)
            rospy.loginfo("Leap Motion tracking started.")
            rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
    finally:
        connection.remove_listener(my_listener)
        rospy.loginfo("Leap Motion listener stopped.")

if __name__ == "__main__":
    main()
