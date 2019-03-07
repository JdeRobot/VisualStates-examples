#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys, threading, time, rospy, signal
from prius_msgs.msg import Control
from sensor_msgs.msg import JointState
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
		rospy.init_node("prius_simulate", anonymous=True, disable_signals=True)

		self.priusPub = rospy.Publisher("/prius_simulate/prius", Control, queue_size=10)
		self.joint_statesSub = rospy.Subscriber("/prius_simulate/joint_states", JointState, self.joint_statesCallback)
		self.joint_states = JointState()
		time.sleep(1) # wait for initialization of the node, subscriber, and publisher

	def stop(self):
		rospy.signal_shutdown("exit ROS node")

	def prius(self, _prius):
		self.priusPub.publish(_prius)



	def joint_statesCallback(self, _joint_states):
		self.joint_states = _joint_states


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
		self.current_time = 0
		self.last_time = 0
		self.last_error = 0
		self.kp = 0.05
		self.ki = 0
		self.kd = 0.03
		self.p = 0
		self.i = 0
		self.d = 0

	def calculate_throttle(self, desired_velocity):
		"""
		PID Throttle Controller
		"""
		feedback_velocity = np.average(np.array(self.globalNamespace.joint_states.velocity[:4])) # 4 Wheels
		error = desired_velocity - feedback_velocity
		print("Error:", error)
		self.current_time = time.time()
		delta_time = self.current_time - self.last_time
		delta_error = error - self.last_error
		self.p = self.kp * error
		self.i += error * delta_time
		self.d = delta_error / delta_time
		self.last_time = self.current_time
		self.last_error = error
		output = self.p + self.ki * self.i + self.kd * self.d
		print("output", output)
		return output

class State1(State):
	def __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.globalNamespace = globalNamespace
		self.namespace = namespace

	def runCode(self):
		output = self.namespace.calculate_throttle(3)
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
		command = Control()
		command.throttle = throttle
		command.brake = brake
		command.steer = 0
		self.globalNamespace.prius(command)


class Namespace1():
	def __init__(self, globalNamespace):
		self.globalNamespace = globalNamespace


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
		gui.addState(1, "stay_on_road", True, 967.0, 920.0, 0)


	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	namespace0 = Namespace0(globalNamespace)
	state0 = State0(0, True, globalNamespace, None, 100, None, gui)
	namespace1 = Namespace1(globalNamespace)
	state1 = State1(1, True, globalNamespace, namespace0, 100, state0, gui)

	state0.startThread()

	while state0.running:
		time.sleep(0.01)

