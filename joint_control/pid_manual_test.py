from pylab import *
from ipywidgets import interact
from IPython import display
from collections import deque

from pid import PIDAgent
agent = PIDAgent()

def set_pid_parameters(kp, ki, kd, delay=0):
    global agent
    agent.joint_controller.Kp = kp
    agent.joint_controller.Ki = ki
    agent.joint_controller.Kd = kd
    agent.joint_controller.set_delay(delay)

joint_name = 'HeadYaw'
sensor = deque(maxlen=100)
target = deque(maxlen=100)

def set_joint_name(name):
    global joint_name
    if joint_name != name:
        joint_name = name
        sensor.clear()
        target.clear()

def set_joint_target(value):
    agent.target_joints[joint_name] = value

# inject plotting input agent's loop
cycle = 0
orig_sense_think_act = agent.sense_think_act
def sense_think_act():
    global cycle
    cycle += 1
    orig_sense_think_act()
    sensor.append(agent.perception.joint[joint_name])
    target.append(agent.target_joints[joint_name])

agent.sense_think_act = sense_think_act
agent.start()
