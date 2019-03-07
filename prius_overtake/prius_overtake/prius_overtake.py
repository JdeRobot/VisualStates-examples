#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys, threading, time, rospy, signal
from prius_msgs.msg import Control
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from visualstates.codegen.python.state import State
from visualstates.codegen.python.temporaltransition import TemporalTransition
from visualstates.codegen.python.conditionaltransition import ConditionalTransition
from visualstates.codegen.python.runtimegui import RunTimeGui
from PyQt5.QtWidgets import QApplication

import numpy as np

globalNamespace = None
state0 = None
app = None

def stopRecursive(state):
	state.stop()
	for childState in state.states:
		stopRecursive(childState)

def sigintHandler(signal, frame):
	global globalNamespace
	global state0
	global app
	if app is not None:		app.quit()
	stopRecursive(state0)
	globalNamespace.stop()

signal.signal(signal.SIGINT, sigintHandler)

class GlobalNamespace():
	def __init__(self):
		rospy.init_node("prius_overtake", anonymous=True, disable_signals=True)

		self.priusPub = rospy.Publisher("/prius_overtake/prius", Control, queue_size=10)
		self.joint_statesSub = rospy.Subscriber("/prius_overtake/joint_states", JointState, self.joint_statesCallback)
		self.joint_states = JointState()
		self.front_left_laserSub = rospy.Subscriber("/prius_overtake/front_left_laser/scan", LaserScan, self.front_left_laserCallback)
		self.front_left_laser = LaserScan()
		self.front_right_laserSub = rospy.Subscriber("/prius_overtake/front_right_laser/scan", LaserScan, self.front_right_laserCallback)
		self.front_right_laser = LaserScan()
		self.current_time = 0
		self.last_time = 0
		self.last_error = 0
		self.kp = 0.04
		self.ki = 0
		self.kd = 0.025
		self.p = 0
		self.i = 0
		self.d = 0

		time.sleep(1) # wait for initialization of the node, subscriber, and publisher

	def stop(self):
		rospy.signal_shutdown("exit ROS node")

	def prius(self, _prius):
		self.priusPub.publish(_prius)



	def joint_statesCallback(self, _joint_states):
		self.joint_states = _joint_states


	def front_left_laserCallback(self, _front_left_laser):
		self.front_left_laser = _front_left_laser


	def front_right_laserCallback(self, _front_right_laser):
		self.front_right_laser = _front_right_laser


	def calculate_throttle(self, desired_velocity):
		"""
		PID Throttle Controller
		"""
		feedback_velocity = np.average(np.array(self.joint_states.velocity[:4])) # 4 Wheels
		error = desired_velocity - feedback_velocity
		self.current_time = time.time()
		delta_time = self.current_time - self.last_time
		delta_error = error - self.last_error
		self.p = self.kp * error
		self.i += error * delta_time
		self.d = delta_error / delta_time
		self.last_time = self.current_time
		self.last_error = error
		output = self.p + self.ki * self.i + self.kd * self.d
		throttle = 0
		brake = 0
		if output > 1:
			throttle = 1
			brake = 0
		elif output > 0:
			throttle = output
			brake = 0
		elif output > -1:
			throttle = 0
			brake = abs(output)
		else:
			throttle = 0
			brake = 1
		return throttle, brake


class State0(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		pass


class Namespace0():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace
		self.detect_car = False

	def getRangesForAngle(self, min_angle, max_angle, laser):
		"""
		Applies Angle Mask to Laser
		"""
		laser_ranges = enumerate(laser.ranges)
		thresh = laser.angle_increment * 180 / 3.14
		ranges = []
		for k, g in enumerate(laser.ranges):
			if k*thresh > min_angle and k*thresh < max_angle:
				ranges.append(g)
		return ranges
	
	def car_in_front(self):
		"""
		Obstacle Detection using Laser Data
		"""
		left_laser = self.globalNamespace.front_left_laser
		right_laser = self.globalNamespace.front_right_laser
		rf_ranges = np.array(self.getRangesForAngle(120, 180, right_laser))
		lf_ranges = np.array(self.getRangesForAngle(0, 60, left_laser))
		if (rf_ranges < 6).any() or (lf_ranges < 6).any():
			self.detect_car = True
			return
		self.detect_car = False
	
	def car_in_right(self):
		"""
		Obstacle Detection using Laser Data
		"""
		right_laser = self.globalNamespace.front_right_laser
		rf_ranges = np.array(self.getRangesForAngle(60, 180, right_laser))
		if(rf_ranges < 8).any():
			self.detect_car = True
			return
		self.detect_car = False
		
	
	

class State1(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		throttle, brake = self.globalNamespace.calculate_throttle(8)
		command = Control()
		command.throttle = throttle
		command.brake = brake
		command.steer = 0
		self.globalNamespace.prius(command)


class Namespace1():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


class State2(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		pass


class Namespace2():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


class State3(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		throttle, brake = self.globalNamespace.calculate_throttle(8)
		command = Control()
		command.throttle = throttle
		command.brake = brake
		command.steer = 0
		self.globalNamespace.prius(command)


class Namespace3():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


class State4(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		pass


class Namespace4():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


class State5(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		throttle, brake = self.globalNamespace.calculate_throttle(8)
		command = Control()
		command.throttle = throttle
		command.brake = brake
		command.steer = 0.6
		self.globalNamespace.prius(command)


class Namespace5():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


class State6(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		throttle, brake = self.globalNamespace.calculate_throttle(8)
		command = Control()
		command.throttle = throttle
		command.brake = brake
		command.steer = -0.6
		self.globalNamespace.prius(command)


class Namespace6():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


class State7(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		throttle, brake = self.globalNamespace.calculate_throttle(8)
		command = Control()
		command.throttle = throttle
		command.brake = brake
		command.steer = -0.6
		self.globalNamespace.prius(command)


class Namespace7():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


class State8(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		throttle, brake = self.globalNamespace.calculate_throttle(8)
		command = Control()
		command.throttle = throttle
		command.brake = brake
		command.steer = 0.6
		self.globalNamespace.prius(command)


class Namespace8():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


class Tran1(ConditionalTransition):
	def __init__(self, id, destinationId, globalNamespace, namespace):
		ConditionalTransition.__init__(self, id, destinationId, globalNamespace, namespace)
	def checkCondition(self):
		self.namespace.car_in_front()
		return self.namespace.detect_car
		

	def runCode(self):
		pass

class Tran2(TemporalTransition):

	def __init__(self, id, destinationId, elapsedTime, globalNamespace, namespace):
		TemporalTransition.__init__(self, id, destinationId, elapsedTime, globalNamespace, namespace)
	def runCode(self):
		pass

class Tran3(ConditionalTransition):
	def __init__(self, id, destinationId, globalNamespace, namespace):
		ConditionalTransition.__init__(self, id, destinationId, globalNamespace, namespace)
	def checkCondition(self):
		self.namespace.car_in_right()
		return not self.namespace.detect_car

	def runCode(self):
		pass

class Tran4(TemporalTransition):

	def __init__(self, id, destinationId, elapsedTime, globalNamespace, namespace):
		TemporalTransition.__init__(self, id, destinationId, elapsedTime, globalNamespace, namespace)
	def runCode(self):
		pass

class Tran5(TemporalTransition):

	def __init__(self, id, destinationId, elapsedTime, globalNamespace, namespace):
		TemporalTransition.__init__(self, id, destinationId, elapsedTime, globalNamespace, namespace)
	def runCode(self):
		pass

class Tran6(TemporalTransition):

	def __init__(self, id, destinationId, elapsedTime, globalNamespace, namespace):
		TemporalTransition.__init__(self, id, destinationId, elapsedTime, globalNamespace, namespace)
	def runCode(self):
		pass

displayGui = False
guiThread = None
gui = None

def readArgs():
	global displayGui
	for arg in sys.argv:
		splitedArg = arg.split('=')
		if splitedArg[0] == '--displaygui':
			if splitedArg[1] == 'True' or splitedArg[1] == 'true':
				displayGui = True
				print('runtime gui enabled')
			else:
				displayGui = False
				print('runtime gui disabled')

def runGui():
	global app
	global gui
	app = QApplication(sys.argv)
	gui = RunTimeGui()
	gui.show()
	app.exec_()

if __name__ == "__main__":

	globalNamespace = GlobalNamespace()


	readArgs()
	if displayGui:
		guiThread = threading.Thread(target=runGui)
		guiThread.start()


	if displayGui:
		while(gui is None):
			time.sleep(0.1)

		gui.addState(0, "root", True, 0.0, 0.0, None)
		gui.addState(1, "stay_on_road", True, 877.0, 895.0, 0)
		gui.addState(2, "change_lane", False, 1090.0, 894.0, 0)
		gui.addState(3, "accelerate", False, 1093.0, 1047.0, 0)
		gui.addState(4, "change_lane", False, 884.0, 1046.0, 0)
		gui.addState(5, "steer_left", True, 983.0, 913.0, 2)
		gui.addState(6, "align", False, 982.0, 1076.0, 2)
		gui.addState(7, "steer_right", True, 972.0, 878.0, 4)
		gui.addState(8, "align", False, 970.0, 1058.0, 4)

		gui.addTransition(1, "detect_car", 1, 2, 984.0, 894.0)
		gui.addTransition(2, "lane_left", 2, 3, 1092.5, 970.0)
		gui.addTransition(3, "car_crossed", 3, 4, 993.0, 1047.0)
		gui.addTransition(4, "success", 4, 1, 880.5, 971.5)
		gui.addTransition(5, "is_align", 5, 6, 984.0, 1001.0)
		gui.addTransition(6, "is_align", 7, 8, 971.0, 968.0)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	namespace0 = Namespace0(globalNamespace)
	state0 = State0(0, True, globalNamespace, None, 100, None, gui)
	namespace1 = Namespace1(globalNamespace)
	state1 = State1(1, True, globalNamespace, namespace0, 100, state0, gui)
	namespace2 = Namespace2(globalNamespace)
	state2 = State2(2, False, globalNamespace, namespace0, 100, state0, gui)
	namespace3 = Namespace3(globalNamespace)
	state3 = State3(3, False, globalNamespace, namespace0, 100, state0, gui)
	namespace4 = Namespace4(globalNamespace)
	state4 = State4(4, False, globalNamespace, namespace0, 100, state0, gui)
	namespace5 = Namespace5(globalNamespace)
	state5 = State5(5, True, globalNamespace, namespace2, 100, state2, gui)
	namespace6 = Namespace6(globalNamespace)
	state6 = State6(6, False, globalNamespace, namespace2, 100, state2, gui)
	namespace7 = Namespace7(globalNamespace)
	state7 = State7(7, True, globalNamespace, namespace4, 100, state4, gui)
	namespace8 = Namespace8(globalNamespace)
	state8 = State8(8, False, globalNamespace, namespace4, 100, state4, gui)

	tran1 = Tran1(1, 2, globalNamespace, namespace0)
	state1.addTransition(tran1)

	tran2 = Tran2(2, 3, 4600, globalNamespace, namespace0)
	state2.addTransition(tran2)

	tran3 = Tran3(3, 4, globalNamespace, namespace0)
	state3.addTransition(tran3)

	tran4 = Tran4(4, 1, 4400, globalNamespace, namespace0)
	state4.addTransition(tran4)

	tran5 = Tran5(5, 6, 2200, globalNamespace, namespace2)
	state5.addTransition(tran5)

	tran6 = Tran6(6, 8, 2100, globalNamespace, namespace4)
	state7.addTransition(tran6)

	state0.startThread()
	state2.startThread()
	state4.startThread()

	while state0.running:
		time.sleep(0.01)

