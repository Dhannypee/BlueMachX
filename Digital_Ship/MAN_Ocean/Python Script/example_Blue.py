import math
import rospy
from dronekit import connect, VehicleMode
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from bluerov_gii.srv import MoveCameraSrv
from bluerov_gii.srv import MoveGripperSrv
from bluerov_gii.msg import Blob 

x_threshold = 70
y_threshold = 90
x_rov = 0
y_rov = 0
r_rov = 0
kp = 0.03
kp2 = 0.14
ki = 0.08
kd = 0.05
error = 0
pTerm = 0
pTerm2 = 0
iTerm = 0
dTerm = 0
oldTerm = 0
oldinput = 0
output = 0
closeness = -5
grab = 0

stop = False
down = False
close = False
distance = False
downlimit = False
Searching = False
Searching2= False
Limit = False


rospy.init_node("example")


def camera_callback(data):
	print ("Camera Position: %s" %data)

def gripper_callback(data):
	print ("Gripper Position: %s" %data)

def bluecolor_callback(data):
	global x_threshold
	global y_threshold
	global x_rov
	global y_rov
	global r_rov
	global pTerm
	global pTerm2
	global iTerm
	global dTerm
	global error
	global error2
	global output
	global oldTerm
	global oldinput
	global grab
	#print("x y r: ",data.x, data.y, data.radius)
	r_rov = data.radius
	x_rov = data.x
	y_rov = data.y	
	error = y_threshold - y_rov.data
	if error > 15:
		down = True 
	pTerm = kp * error
	iTerm = oldTerm + (ki * error)
	dTerm = kd * (y_rov.data - oldinput)
	if iTerm > 1:
		iTerm = 1
	elif iTerm < -1: 
		iTerm = -1
	oldTerm = iTerm
	oldinput = y_rov.data
	output = pTerm   + iTerm
	if output > 1:
		output = 1
	

rospy.Subscriber("/Gripper_Position", Int32,gripper_callback)

rospy.Subscriber("/cameraPosition", Int32,camera_callback)

rospy.Subscriber("/color/blue/blob", Blob, bluecolor_callback)


move_camera_srv=rospy.ServiceProxy("/moveCamera",MoveCameraSrv)
move_gripper_srv=rospy.ServiceProxy("/moveGripper",MoveGripperSrv)


# Connect to the Vehicle autopilot.
connection_string="127.0.0.1:14550"
print("Connecting to vehicle on: %s" % (connection_string,))
autopilot = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
print ("Get some vehicle attribute values:")
print (" GPS: %s" % autopilot.gps_0)
print (" Battery: %s" % autopilot.battery)
print (" System status: %s" % autopilot.system_status.state)
print (" Mode: %s" % autopilot.mode.name )

# Arm to move the robot
autopilot.armed=True
print ("%s" % autopilot.armed)

#move_camera_srv(Int32(5))
#rospy.sleep(0.5)
move_camera_srv(Int32(24))
rospy.sleep(0.5)
move_gripper_srv(Int32(0))
rospy.sleep(0.5)





	# Motor values
	# 1000 reverse full speed
	# 1500 middle (off)
	# 2000 forward full speed

	# Motor channels for vehicle control
	#1 Pitch
	#2 Roll
	#3 Throttle
	#4 Yaw
	#5 Forward
	#6 Lateral


while autopilot.armed == True:
	if r_rov.data <= 0 and Searching == False:
		print("Searching for object")
		if r_rov.data > 0:
			Searching == True	#Stop, Move Camera and start searching for the colour of the object
		autopilot.channels.overrides['5'] = 1500 
		rospy.sleep(1)
		print("Searching for object")
		move_camera_srv(Int32(25))
		rospy.sleep(0.5)
		autopilot.channels.overrides['5'] = 1450 
		rospy.sleep(1)
	elif r_rov.data < x_threshold and stop == False:
		print("r_rov", r_rov.data)
		print("x_rov", x_rov.data)
		print("y_rov", y_rov.data)
		print("Object Found")
		print("moving forward")
		if r_rov.data > x_threshold:
			stop = True
		print("moving forward")
		autopilot.channels.overrides['5'] = 1580 
		rospy.sleep(0.05)
	else:
		Limit = True
		if Limit == True:
			Searching = True
			stop = True
		else:
			Searching = False
			stop = False
		print("Blue_ROV is stopped: Object Threshold is reached")
		print("r_rov", r_rov.data)
		print("stop", stop)
		print("Searching: ", Searching)
		autopilot.channels.overrides['5'] = 1500
		rospy.sleep(1)
		while down == False:
			print("going down to align to object centre") 
			print("y_rov", y_rov.data)
			print("error", error)
			print("Output", output)
			print("r_rov", r_rov.data)
			autopilot.channels.overrides['3'] = output + 1500
			rospy.sleep(0.5)
			if y_rov.data > 100 and y_rov.data < 200:
				print("Blue_ROV about to go grab object")
				if r_rov.data <100  and  close == False:
					if r_rov.data >100:
						close = True
						print("object reached")
					print("moving close to object")
					print("r_rov", r_rov.data)
					print("y_rov", y_rov.data)
					autopilot.channels.overrides['5'] = 1515 
					rospy.sleep(1)
					move_gripper_srv(Int32(35))
					rospy.sleep(0.05)
			elif r_rov.data > 150:
					print("Grabing object")
					move_gripper_srv(Int32(0))
					rospy.sleep(0.5)
					autopilot.channels.overrides['5'] = 1490
					rospy.sleep(0.005)
			elif  r_rov.data <= 0 and Searching2== False:
				print("Searching for object")
				if r_rov.data > 0:
					Searching2== True	#Stop, Move Camera and start searching for the colour of the object
				autopilot.channels.overrides['5'] = 1500 
				rospy.sleep(1)
				print("Searching for object")
				move_camera_srv(Int32(25))
				rospy.sleep(0.5)
				autopilot.channels.overrides['5'] = 1450 
				rospy.sleep(1)
rospy.spin()

