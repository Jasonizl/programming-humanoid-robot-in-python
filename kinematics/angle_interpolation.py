'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.timePoint = 0
        self.isInterpolated = False
        self.spline = []
        self.startTime = -1
        self.currentTime = -1
        self.endTime = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        # How do we want to it?
        # 1.    Circle through all the joints in the keyframe [Works]
        # 2.    On every keyframe, circle through all times [Works]
        # 3.    interpolation in this part [Works]
        # 4.    extract coefficients from spline [Works]
        # 5.    calculate interpolated value for specific time [works]
        # (6.   check if we have ot apply because time is right now?)

        if self.spline == [] and self.timePoint == 0:
            self.timePoint = np.zeros(len(keyframes[0]), dtype=int)

        if (self.isInterpolated):
            # update time
            if(self.startTime == -1):
                self.startTime = perception.time

            self.currentTime = perception.time

            dif = self.currentTime - self.startTime
            
            # update every joint in keyframe
            for i in range(len(keyframes[0])):
                if dif > keyframes[1][i][self.timePoint[i]]:
                    if len(keyframes[1][i])-1 > self.timePoint[i] + 1:
                        self.timePoint[i] += 1
                if np.max(keyframes[1][i]) < dif:
                    target_joints[keyframes[0][i]] = 0
                else:
                    target_joints[keyframes[0][i]] = self.spline[i][self.timePoint[i]](dif)

            return target_joints

        # 1. Circle through all joints
        for i in range(len(keyframes[0])):
            x = np.array(keyframes[1][i]) # = times
            y_tmp = keyframes[2][i] # values at times (have to extract only time don't care for bezier)
            y = np.ndarray(len(y_tmp)) # = values for interpolation

            # extract first values of y_tmp
            for j in range(0, len(y_tmp)):
                y[j] = y_tmp[j][0]

            # preparation for spline
            size = x.size - 1
            A_matrix = np.zeros((4*size, 4*size))
            b_matrix = np.zeros(4*size)

            # 2. On every keyframe, circle through all times
            for j in range(0, size):
                # 3. interpolation in this part
                splineX = j*4

                A_matrix[splineX, splineX:splineX+4] = np.array([1, x[j], x[j]**2, x[j]**3])
                b_matrix[splineX] = y[j]

                A_matrix[splineX + 1, splineX:splineX + 4] = np.array([1, x[j + 1], x[j + 1] ** 2, x[j + 1] ** 3])
                b_matrix[splineX + 1] = y[j + 1]

                if j == size - 1:
                    A_matrix[splineX + 2, splineX:splineX + 4] = np.array([0, 0, 2, 6 * x[size]])  # f''(x_size) = 0
                    A_matrix[splineX + 3, 0:4] = np.array([0, 0, 2, 6 * x[0]])  # f''(x0) = 0
                else:
                    A_matrix[splineX + 2, splineX:splineX + 4] = np.array([0, 1, 2 * x[j + 1], 3 * x[j + 1] ** 2])
                    A_matrix[splineX + 2, splineX + 4:splineX + 8] = np.array([0, -1, -2 * x[j + 1], -3 * x[j + 1] ** 2])

                    A_matrix[splineX + 3, splineX:splineX + 4] = np.array([0, 0, 2, 6 * x[j + 1]])
                    A_matrix[splineX + 3, splineX + 4:splineX + 8] = np.array([0, 0, -2, -6 * x[j + 1]])

            solution = np.linalg.solve(A_matrix, b_matrix)

            spline = []
            # 4. extract coefficients from solution
            for j in range(0, solution.size, 4):
                spline.append(np.poly1d([solution[j + 3], solution[j + 2], solution[j + 1], solution[j]]))

            # save our spline into the agent for later use
            self.spline.append(spline)

        self.isInterpolated = True

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
