from std_msgs.msg import Float64MultiArray
import rospy

rospy.init_node('pose_publisher')
pub = rospy.Publisher('/target_6d_pose', Float64MultiArray, queue_size=1)

msg = Float64MultiArray()
msg.data = [0.2, 0.0, 0.25, 0.0, 0.0, 0.0]  # x, y, z, roll, pitch, yaw
pub.publish(msg)