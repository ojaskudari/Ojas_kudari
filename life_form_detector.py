#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS		SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class swift():
	"""docstring for swift"""
	def __init__(self):

		
		
		


		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.current_setpoint = [-6,-6,20]
		self.setpoint = True
        #     [0, 0, 23],
        #     [2, 0, 23],
        #     [2, 2, 23],
        #     [2, 2, 25],
        #     [-5, 2, 25],
        #     [-5, -3, 25],
        #     [-5, -3, 21],
        #     [7, -3, 21],
        #     [7, 0, 21],
        #     [0, 0, 19]]
		# self.setpoint_index = 1
		# self.current_setpoint = self.setpoint[self.setpoint_index]

		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [9.18, 3.36,3.96]
		self.Ki = [0.0032, 0.00032,0.0006]
		self.Kd = [30,9,175.8]
		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.alt_error = 0.0 #variables for calculting error for PID
		self.prev_alt_error = 0.0 
		self.sum_alt_error = 0.0

		self.min_throttle = 1000
		self.max_throttle = 2000

		self.pitch_error = 0.0
		self.prev_pitch_error = 0.0
		self.sum_pitch_error =0.0

		self.min_pitch = 1000
		self.max_pitch = 2000

		self.roll_error = 0.0
		self.prev_roll_error = 0.0
		self.sum_roll_error =0.0

		self.min_roll = 1000
		self.max_roll = 2000






		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms



		self.sample_time = 0.03333 # in seconds 30hz




		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub = rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error',Float64,queue_size=1)



	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/swift/camera_rgb/image_raw',Image,self.image_callback)



		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE

	def show_image(img):
		cv2.imshow('Image Window',img)
		cv2.waitKey(3)
		
	def image_callback(self,img_msg):

		bridge = CvBridge()
		rospy.loginfo(img_msg.header)

		try:
			cv_image = bridge.imgmsg_to_cv2(img_msg,"passthrough")
		except CvBridgeError:
			rospy.logerr("CvBridgeError: {0}".format(e))


		image = cv2.imread(cv_image,1)
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(gray, ksize=(11, 11), sigmaX=cv2.BORDER_DEFAULT)


		_, mask = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

# Find contours in the binary mask
		contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		centroid_list = []
		area_list = []

		for contour in contours:
    # Calculate the area of the contour
			area = cv2.contourArea(contour)

    # Calculate the centroid of the contour
			M = cv2.moments(contour)
			if M["m00"] != 0:
				cx = int(M["m10"] / M["m00"])
				cy = int(M["m01"] / M["m00"])
			else:
				cx, cy = 0, 0

			centroid_list.append((cx, cy))
			area_list.append(area)

    
			x, y, w, h = cv2.boundingRect(contour)
			cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
			show_image(cv_image)

		cv2.imwrite("led_detection_results.png", cv_image) 
	#cv2.namedWindow("Image window",0)	

	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0008
		self.Kd[2] = alt.Kd * 0.3
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,pitch):
		 self.Kp[1] = pitch.Kp*0.06
		 self.Ki[1] = pitch.Ki*0.0008
		 self.Kd[1] = pitch.Kd*0.3


	def roll_set_pid(self,roll):
		 self.Kp[0] = roll.Kp*0.06
		 self.Ki[0] = roll.Ki*0.0008
		 self.Kd[0] = roll.Kd*0.3










	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		#for z throttle

		self.alt_error = -(self.current_setpoint[2] - self.drone_position[2]) #error for z axis -sign is due current z position of drone is -17 

		#self.iterm = (self.iterm+self.alt_error)*self.Ki[2]
		self.cmd.rcThrottle = int(1586.2 + (self.alt_error*self.Kp[2]) + ((self.alt_error - self.prev_alt_error)*self.Kd[2])+ self.sum_alt_error*self.Ki[2])#complete the formula

		if self.cmd.rcThrottle>2000:
			self.cmd.rcThrottle=2000 #limiting the rc throttle between 1000 to 2000

		if self.cmd.rcThrottle<1000:
			self.cmd.rcThrottle=1000

		self.prev_alt_error = self.alt_error

		self.sum_alt_error = self.sum_alt_error + self.alt_error

#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		#for y pitch

		self.pitch_error = -(self.current_setpoint[1] - self.drone_position[1])

		self.cmd.rcPitch = int(1500+(self.pitch_error*self.Kp[1]) + ((self.pitch_error - self.prev_pitch_error)*self.Kd[1])+self.sum_pitch_error*self.Ki[1])

		if self.cmd.rcPitch>2000:
			self.cmd.rcPitch=2000

		if self.cmd.rcPitch<1000:
			self.cmd.rcPitch=1000


		self.prev_pitch_error = self.pitch_error

		self.sum_pitch_error = self.sum_pitch_error + self.pitch_error

#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		#for z roll

		self.roll_error = (self.current_setpoint[0] - self.drone_position[0])

		self.cmd.rcRoll = int(1500+(self.roll_error*self.Kp[0]) + ((self.roll_error - self.prev_roll_error)*self.Kd[0])+self.sum_roll_error*self.Ki[0])

		if self.cmd.rcRoll>2000:
			self.cmd.rcRoll=2000

		if self.cmd.rcRoll<1000:
			self.cmd.rcRoll=1000


		self.prev_roll_error = self.roll_error

		self.sum_roll_error = self.sum_roll_error + self.roll_error







	#------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.alt_error)
		self.pitch_error_pub.publish(self.pitch_error)
		self.roll_error_pub.publish(self.roll_error)
	
	def run(self):
        
		if self.setpoint==True:
			self.current_setpoint=[-5,-6,20]


            # #     # # Check if the drone is close to the current waypoint
			distance_to_setpoint = ((self.current_setpoint[0] - self.drone_position[0]) ** 2 +
                                    (self.current_setpoint[1] - self.drone_position[1]) ** 2) ** 0.5

             
			# if distance_to_setpoint < 0.6:
			# 	if self.current_setpoint[0] % 3 ==0:
			# 		if self.current_setpoint[1]<6:
			# 			self.current_setpoint[1]+=3

			# 		if self.current_setpoint[1] == 6:
			# 			self.current_setpoint[0] += 2
			# 	else:
			# 		if self.current_setpoint[1]>-6:
			# 			self.current_setpoint[1] -= 3

			# 		if self.current_setpoint[1] == -6:
			# 			self.current_setpoint[0] += 2


				# if self.current_setpoint[0] == 9:
				# 	self.setpoint = False




if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.run()
		swift_drone.pid()
		r.sleep()
