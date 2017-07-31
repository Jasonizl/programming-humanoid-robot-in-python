'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import SimpleXMLRPCServer
import os
import sys
import threading
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
import xmlrpclib
from inverse_kinematics import InverseKinematicsAgent

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        retVal = self.target_joints.get(joint_name)
        if retVal == None:
            return "Joint not found."

        return retVal
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        if self.target_joints.get(joint_name) == None:
            return "Joint not found."

        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        post = self.recognize_posture(self.perception)
        return str(post)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.angle_interpolation(keyframes, self.perception)
        return

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        transform = self.chains[name]
        if transform == None:
            return "No transform with given name."
        return str(transform)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        keyframes = self.set_transforms(self, effector_name, transform)
        self.execute_keyframes(keyframes)


if __name__ == '__main__':
    # https://docs.python.org/2/library/xmlrpclib.html
    agent = ServerAgent()

    server = SimpleXMLRPCServer.SimpleXMLRPCServer(('localhost', 9000), logRequests=True, allow_none=True)
    server.register_instance(agent)
    server.register_introspection_functions()
    server.register_multicall_functions()

    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    print "Server started successfully"
    print "Running on \"http://localhost:9000\"\n"
    print "Starting agent..."
    agent.run()

