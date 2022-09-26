#!/usr/bin/env python
import rospy

from std_msgs.msg import String   #other messages -geom message-pose dependencies
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf import TransformListener

import serial
import time
import math

serdev = '/dev/ttyACM0' # serial device of JeVois
ser = serial.Serial(serdev, 115200, timeout=1) #porta serial ser

def config_camera():
	# Receive data from USB
	ser.write("setpar serout USB\r".encode())
	out = ser.readline().rstrip()  #.readline() reads the first line from port
	#some_string.strip() called without parameters removes all whitespace from the start and end of some_string
	print(out)

	# Initialize DemoArUco module
	# ser.write("setmapping2 YUYV 640 480 20.0 JeVois DemoArUco\r".encode())
	ser.write("setmapping2 YUYV 320 240 30.0 JeVois DemoArUco\r".encode())
	
	out = ser.readline().rstrip()
	print(out)

	# Set 3D pose messages
	ser.write("setpar dopose true\r".encode())
	out = ser.readline().rstrip()
	print(out)

	# Set 3D pose messages
	ser.write("setpar markerlen 51 \r".encode())
	out = ser.readline().rstrip()
	print(out)

	# Set 3D pose messages
	ser.write("setpar serprec 4\r".encode())
	out = ser.readline().rstrip()
	print(out)

	# Set Detail style messages
	ser.write("setpar serstyle Detail\r".encode())
	out = ser.readline().rstrip()
	print(out)

	# Set Detail style messages
	ser.write("setpar serstamp FrameTime\r".encode())
	out = ser.readline().rstrip()
	print(out)

	# # Start image stream
	ser.write("streamon\r".encode())
	out = ser.readline().rstrip()
	print(out)

	time.sleep(1)
 
def euler_from_quaternion(x, y, z, w):
	"""
	Convert a quaternion into euler angles (roll, pitch, yaw)
	roll is rotation around x in radians (counterclockwise)
	pitch is rotation around y in radians (counterclockwise)
	yaw is rotation around z in radians (counterclockwise)
	"""
	t0 = +2.0 * (w * x + y * z)  
	t1 = +1.0 - 2.0 * (x * x + y * y)  
	roll_x = math.atan2(t0, t1) * 180.0 / math.pi  #sai em graus; conta em rad 

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2) * 180.0 / math.pi

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4) * 180.0 / math.pi

	return roll_x, pitch_y, yaw_z # in radians

# def transferencia(posi):
# 		tfl = TransformListener()
    
# 		to_link='base_link'
# 		from_link='link_3'

		
# 		#rospy.sleep(5)
    
# 		t = rospy.Time.now()

# 		rospy.sleep(6)

# 		mpose_transf = None

# 		rospy.loginfo('Waiting for transform for some time...')

# 		tfl.waitForTransform(to_link,from_link,t,rospy.Duration(5))

# 		if tfl.canTransform(to_link,from_link,t):

# 			mpose_transf = tfl.transformPose(to_link,posi)
# 			print (mpose_transf)

# 		else:
# 			rospy.logerr('Transformation is not possible!')
# 			sys.exit(0)


def talker():
	pub = rospy.Publisher('/end_effector_jevois_pose', PoseStamped, queue_size=10)
	rospy.init_node('jevois_pose', anonymous=True)
	rate = rospy.Rate(150) # 10hz
	config_camera()

	# from_link = '/gripper'
	# to_link = '/base_link'
    
	while not rospy.is_shutdown():
		

		# Read data continuously
		
		# Read a whole line and strip any trailing line ending character:
		line = ser.readline().rstrip()
		line= line.decode("utf-8") 
		# print ("received: {}".format(line))

		# Split the line into tokens:
		tok = line.split()
		#split() breaks the string considering white spaces

		# Skip if timeout or malformed line:
		if len(tok) < 1: continue

	    #print(len(tok))

		# Skip if not a standardized "Detail 3D" message:
		# See http://jevois.org/doc/UserSerialStyle.html
		if tok[1] != 'D3': continue
		# From now on, we hence expect: D3 id x y z w h d q1 q2 q3 q4
		if len(tok) != 13: continue

		# Assign some named Python variables to the tokens:
		date,D, id, x, y, z, w, h, d, q1, q2, q3, q4 = tok

		norma= math.sqrt( (float(q1))**2 + (float(q2))**2+ (float(q3))**2 +(float(q4))**2)
		n_q1= (float(q1))/ norma
		n_q2= (float(q2))/ norma
		n_q3= (float(q3))/ norma
		n_q4= (float(q4))/ norma

		roll, pitch, yaw = euler_from_quaternion(float(n_q1), float(n_q2), float(n_q3), float(n_q4))

		#print(id)
		#print(x)
		# print("Found w,h,d ({:.2f},{:.2f},{:.2f}) quaternions ({:.2f},{:.2f},{:.2f},{:.2f})".format(float(w), float(h), float(d), float(q1), float(q2), float(q3), float(q4)))
		print("Found ArUco {} at ({:.2f},{:.2f},{:.2f}) quaternions ({:.2f},{:.2f},{:.2f},{:.2f}) euler angles ({:.2f},{:.2f},{:.2f})".format(id, float(x), float(y), float(z), float(n_q1), float(n_q2), float(n_q3), float(n_q4),roll, pitch, yaw))


		goal= PoseStamped()

		#definir goal.header.seq
		#definir goal.header.stamp
		goal.header.stamp = rospy.Time.now(); 
		#definir goal.header.frame_id
		goal.header.frame_id = '/camera'

		goal.pose.position.x = float(x)
		goal.pose.position.y = float(y)
		goal.pose.position.z = float(z)

		goal.pose.orientation.x =  float(n_q1)
		goal.pose.orientation.y = float(n_q2)
		goal.pose.orientation.z =float(n_q3)
		goal.pose.orientation.w = float(n_q4)


		pub.publish(goal)
		# transferencia(goal)
		rate.sleep()

	
		
if __name__ == "__main__":
	try:
		talker()
	except rospy.ROSInterruptException:
		pass



