'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from numpy import *


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        target = matrix([transform])
        while True:
            T = self.forward_kinematics(effector_name)
            Te = matrix([self.from_trans(T[-1])])
            e = target - Te
            T = matrix([self.from_trans(i) for i in T[1:-1]])
            J = Te - T
            J = J.T
            J[-1, :] = 1
            JJT = J * J.T
            d_theta = 0.001 * J.T * JJT.I * e.T
            # print (d_theta.T * d_theta)[0, 0]
            joint_angles += asarray(d_theta.T)[0]
            if (d_theta.T * d_theta)[0, 0] < 1e-6:
                break


        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

    # from ipynb
    def from_trans(m):
        '''get x, y, theta from transform matrix'''
        return [m[0, -1], m[1, -1], np.atan2(m[1, 0], m[0, 0])]

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()