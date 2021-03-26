#!/usr/bin/env python
import rospy
import numpy as np
from navio2ros.msg import RC # for reading in RC values from TX
from navio2ros.msg import PWM # for outputting values to the servo rail
from navio2ros.msg import ADC
from navio2ros.msg import PID
from navio2ros.msg import IMU
from navio2ros.msg import AHRS
from navio2ros.msg import Angular
import complete_functions as func

# setup a subscriber, for this assignment we only want IMU
sub_madgwick = rospy.Subscriber('/madgwickpub', AHRS, func.callback_madgwick)

# setup a subscriber, for this assignment we only want IMU
sub_imu = rospy.Subscriber('/imumpupub', IMU, func.callback_imu)

# setup a subscriber, for this assignment we only want IMU
sub_adc = rospy.Subscriber('/adcpub', ADC, func.callback_adc)

sub_rc = rospy.Subscriber('/rcpub', RC, func.callback_rc, queue_size=10)

# setup a publisher, for this assignment we only want IMU
pub_pid = rospy.Publisher('/pid', PID, queue_size=15)

pub_pwm = rospy.Publisher('/motorcommand', PWM, queue_size=10)

# register the node
rospy.init_node('normal_cascade', anonymous=True)

#rcin = RC() # not currently using, should be used to get other RC channels
pwmout = PWM()
pid = PID()

#Setting initial error values to zero
erollrate = np.zeros(2)
derollrate = 0
ierollrate = np.zeros(2)
traprollrate = 0
erollangle = np.zeros(2)
derollangle = 0
ierollangle = np.zeros(2)
traprollangle = 0

epitchrate = np.zeros(2)
depitchrate = 0
iepitchrate = np.zeros(2)
trappitchrate = 0
epitchangle = np.zeros(2)
depitchangle = 0
iepitchangle = np.zeros(2)
trappitchangle = 0

eyawrate = np.zeros(2)
deyawrate = 0
ieyawrate = np.zeros(2)
trapyawrate = 0

#Initalizing PID for roll and rate
Prollrate = 0
Drollrate = 0
Irollrate = 0
Prollangle = 0
Drollangle = 0
Irollangle = 0

Ppitchrate = 0
Dpitchrate = 0
Ipitchrate = 0
Ppitchangle = 0
Dpitchangle = 0
Ipitchangle = 0

Pyawrate = 0
Dyawrate = 0
Iyawrate = 0

#Setting kp, kd, ki for rate controller (40% Baseline Throttle)
kprollrate = 1.299
kdrollrate = .0085
kirollrate = 0.25
kprollangle = 3.0
kdrollangle = 0.006
kirollangle = 0


kppitchrate = 1.299
kdpitchrate = 0.0085
kipitchrate = .25
kppitchangle = 2.25
kdpitchangle = 0.00175
kipitchangle = 0

kpyawrate = 1.75
kdyawrate = 0
kiyawrate = 0

#Set commands to zero
rollcmd = 0
pitchcmd = 0
yawcmd = 0

#Set initial arrays to zero
uf_rollrate = np.zeros(4)
uf_pitchrate = np.zeros(4)
uf_yawrate = np.zeros(4)
uf_rollangle = np.zeros(4)
uf_pitchangle = np.zeros(4)

#Set limits so commands dont go crazy due to noise
rollpitchrate_ceiling = 175
rollpitchangle_ceiling = 50
yawrate_ceiling = 100

# human pilot cannot change sticks faster than 50Hz
rate = rospy.Rate(100)

if __name__ == '__main__':

	# try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
	try:
		while not rospy.is_shutdown():

			if func.killswitch > 1500:
				
				#Resets integral error with kill switch
				ierollrate = np.zeros(2)
                		iepitchrate = np.zeros(2)
                		ieyawrate = np.zeros(2)
				ierollangle = np.zeros(2)
                		iepitchangle = np.zeros(2)
                		ieyawangle = np.zeros(2)

    		    		for i in range(len(pwmout.channel)):
				    pwmout.channel[i] = 1.0 # rc values are integers (1000-2000), we want 1.0-2.0
				
				# publish the topic to motor command
				pub_pwm.publish(pwmout)

				rate.sleep()

			elif func.killswitch < 1500:
				
				#Calculating the dt, Error, Derivative Error, and integral error for Roll
				dt = .01

				#Calculating the dt, Error, Derivative Error, and integral error for Roll and Pitch Angles
				roll_angle_desired = func.rc_roll
				#uf_rollangle[0] = func.roll_angle
				roll_angle_actual = func.roll_angle#func.movingaverage(uf_rollangle[0], uf_rollangle[1], uf_rollangle[2], uf_rollangle[3])
				erollangle[0] = roll_angle_desired - roll_angle_actual
				derollangle = (erollangle[0]-erollangle[1])/dt
				traprollangle = ((erollangle[0]+erollangle[1])/2.0)*dt
				ierollangle[0] = ierollangle[1]+traprollangle
                
				pitch_angle_desired = func.rc_pitch
				#uf_pitchangle[0] = func.pitch_angle
				pitch_angle_actual = func.pitch_angle#func.movingaverage(uf_pitchangle[0], uf_pitchangle[1], uf_pitchangle[2], uf_pitchangle[3])
                		epitchangle[0] = pitch_angle_desired - pitch_angle_actual
				depitchangle = (epitchangle[0]-epitchangle[1])/dt
				trappitchangle = ((epitchangle[0]+epitchangle[1])/2.0)*dt
				iepitchangle[0] = iepitchangle[1]+trappitchangle

				#Calculate PID for Roll
				Prollangle = kprollangle*erollangle[0]
				Drollangle = kdrollangle*derollangle
				Irollangle = kirollangle*ierollangle[0]
               			roll_rate_cmd = Prollangle + Drollangle + Irollangle

                		Ppitchangle = kppitchangle*epitchangle[0]
				Dpitchangle = kdpitchangle*depitchangle
				Ipitchangle = kipitchangle*iepitchangle[0]
               			pitch_rate_cmd = Ppitchangle + Dpitchangle + Ipitchangle

				#print(roll_angle_desired, pitch_angle_desired)

				#Cascade into rate controller
				roll_rate_desired = roll_rate_cmd
				uf_rollrate[0] = func.rate_roll
				roll_rate_actual = func.movingaverage(uf_rollrate[0], uf_rollrate[1], uf_rollrate[2], uf_rollrate[3])

                		pitch_rate_desired = pitch_rate_cmd
				uf_pitchrate[0] = func.rate_pitch
				pitch_rate_actual = func.movingaverage(uf_pitchrate[0], uf_pitchrate[1], uf_pitchrate[2], uf_pitchrate[3])
                
                		yaw_rate_desired = func.rc_yawrate
				uf_yawrate[0] = func.rate_yaw
				yaw_rate_actual = func.movingaverage(uf_yawrate[0], uf_yawrate[1], uf_yawrate[2], uf_yawrate[3])

				#Setting ceilings so the motors dont max out
				if abs(roll_rate_actual) > rollpitchrate_ceiling:
					if roll_rate_actual > 0:
						roll_rate_actual = rollpitchrate_ceiling
					if roll_rate_actual < 0:
						roll_rate_actual = -rollpitchrate_ceiling

				if abs(pitch_rate_actual) > rollpitchrate_ceiling:
					if pitch_rate_actual > 0:
						pitch_rate_actual = rollpitchrate_ceiling
					if pitch_rate_actual < 0:
						pitch_rate_actual = -rollpitchrate_ceiling

				if abs(yaw_rate_actual) > yawrate_ceiling:
					if yaw_rate_actual > 0:
						yaw_rate_actual = yawrate_ceiling
					if yaw_rate_actual < 0:
						yaw_rate_actual = -yawrate_ceiling

				#Calculating the dt, Error, Derivative Error, and integral error for Roll, pitch and yaw rate
				erollrate[0] = roll_rate_desired - roll_rate_actual
				derollrate = (erollrate[0]-erollrate[1])/dt
				traprollrate = ((erollrate[0]+erollrate[1])/2.0)*dt
				ierollrate[0] = ierollrate[1]+traprollrate
                
                		epitchrate[0] = pitch_rate_desired - pitch_rate_actual
				depitchrate = (epitchrate[0]-epitchrate[1])/dt
				trappitchrate = ((epitchrate[0]+epitchrate[1])/2.0)*dt
				iepitchrate[0] = iepitchrate[1]+trappitchrate
                
                		eyawrate[0] = yaw_rate_desired - yaw_rate_actual
				deyawrate = (eyawrate[0]-eyawrate[1])/dt
				trapyawrate = ((eyawrate[0]+eyawrate[1])/2.0)*dt
				ieyawrate[0] = ieyawrate[1]+trapyawrate
				
				#Calculate PID for Roll
				Prollrate = kprollrate*erollrate[0]
				Drollrate = kdrollrate*derollrate
				Irollrate = kirollrate*ierollrate[0]
               
                		Ppitchrate = kppitchrate*epitchrate[0]
				Dpitchrate = kdpitchrate*depitchrate
				Ipitchrate = kipitchrate*iepitchrate[0]
				
                		Pyawrate = kpyawrate*eyawrate[0]
				Dyawrate = kdyawrate*deyawrate
				Iyawrate = kiyawrate*ieyawrate[0]
                
                		#Calculate Motor Commands for Roll and Pitch
				rollcmd = Prollrate + Drollrate + Irollrate
				pitchcmd = Ppitchrate + Dpitchrate + Ipitchrate				
				yawcmd = Pyawrate + Dyawrate + Iyawrate
 
				#Setting Commands for motor (unscaled) 
				M1_cmd = func.rc_throttle - pitchcmd - yawcmd
				pwmout.channel[0] = M1_cmd/1000 
				M2_cmd = func.rc_throttle - rollcmd + yawcmd
				pwmout.channel[1] = M2_cmd/1000
                		M3_cmd = func.rc_throttle + pitchcmd - yawcmd
				pwmout.channel[2] = M3_cmd/1000
                		M4_cmd = func.rc_throttle + rollcmd + yawcmd
				pwmout.channel[3] = M4_cmd/1000

				#Update values before iterating again			
				erollrate[1] = erollrate[0]
				ierollrate[1] = ierollrate[0]

				erollangle[1] = erollangle[0]
				ierollangle[1] = ierollangle[0]

				uf_rollrate[3] = uf_rollrate[2]
				uf_rollrate[2] = uf_rollrate[1]
				uf_rollrate[1] = uf_rollrate[0]

				uf_rollangle[3] = uf_rollangle[2]
				uf_rollangle[2] = uf_rollangle[1]
				uf_rollangle[1] = uf_rollangle[0]

                		epitchrate[1] = epitchrate[0]
				iepitchrate[1] = iepitchrate[0]

                		epitchangle[1] = epitchangle[0]
				iepitchangle[1] = iepitchangle[0]

				uf_pitchrate[3] = uf_pitchrate[2]
				uf_pitchrate[2] = uf_pitchrate[1]
				uf_pitchrate[1] = uf_pitchrate[0]

				uf_pitchangle[3] = uf_pitchangle[2]
				uf_pitchangle[2] = uf_pitchangle[1]
				uf_pitchangle[1] = uf_pitchangle[0]

				uf_yawrate[3] = uf_yawrate[2]
				uf_yawrate[2] = uf_yawrate[1]
				uf_yawrate[1] = uf_yawrate[0]

                		eyawrate[1] = eyawrate[0]
				ieyawrate[1] = ieyawrate[0]
				
				#Set values to output on the PID message
				pid.desired_roll_rate = func.rc_rollrate
				pid.actual_roll_rate = func.rate_roll
				pid.rollcmd = rollcmd
				pid.prollangle = Prollangle
				pid.drollangle = Drollangle
				pid.irollangle = Irollangle
                
				pid.desired_pitch_rate = func.rc_pitchrate
				pid.actual_pitch_rate = func.rate_pitch
				pid.pitchcmd = pitchcmd
				pid.ppitchangle = Ppitchangle
				pid.dpitchangle = Dpitchangle
				pid.ipitchangle = Ipitchangle
                
				pid.desired_yaw_rate = func.rc_yawrate
				pid.actual_yaw_rate = func.rate_yaw
				pid.yawcmd = yawcmd
 				pid.pyawrate = Pyawrate
				pid.dyawrate = Dyawrate
				pid.iyawrate = Iyawrate
				
				pub_pid.publish(pid)

				# write the pwmout values, using outval as the commmand for all channels
				for i in range(len(pwmout.channel)):
				    pwmout.channel[i] = pwmout.channel[i] # rc values are integers (1000-2000), we want 1.0-2.0

				pub_pwm.publish(pwmout)

				rate.sleep()


	# as stated before, try/except is used to nicely quit the program using ctrl+c
	except rospy.ROSInterruptException:

		# before shutting down, turn all outputs back to 1 for safety
		for i in range(len(pwmout.channel)):
			pwmout.channel[i] = 1.0

		# publish the topic before closing
		pub_pwm.publish(pwmout)

		pass
