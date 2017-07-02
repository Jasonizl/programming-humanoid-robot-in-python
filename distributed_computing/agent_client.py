'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpclib
import threading


class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''

    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.execute_keyframes(keyframes))
        thread.start()  # starts new thread and doesn't block further input

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.set_transform(effector_name, transform))
        thread.start()  # starts new thread and doesn't block further input


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''

    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.MyProxy = xmlrpclib.ServerProxy("http://localhost:9000")  # connect to server

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        retVal = self.MyProxy.get_angle(joint_name)
        if retVal != None:
            print retVal

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        retVal = self.MyProxy.set_angle(joint_name, angle)
        if retVal != None:
            print retVal

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        self.MyProxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.MyProxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        retval = self.MyProxy.get_transform(name)
        print retval

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.MyProxy.set_transform(effector_name, transform)


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    # get_angle
    agent.get_angle("HeadPitch")  # should return inital value 0
    agent.get_angle("HeadHeadHead")  # should not work

    # set_angle
    agent.set_angle("HeadPitch", 0.5)  # should set angle of HeadPitch
    agent.set_angle("HeadHeadHead", 0.5)  # should not work
    agent.get_angle("HeadPitch")  # should return ~ 0.5

    # agent.get_transform("test")
    agent.get_transform("Head")

