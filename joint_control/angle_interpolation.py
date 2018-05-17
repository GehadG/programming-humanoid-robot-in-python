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
from keyframes import *


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def calculate_bezier_interpolation(self, p_0, p_1, p_2, p_3, t):

        b_0 = ((1 - t)**3) * p_0
        b_1 = 3 * ((1 - t)**2) * t * p_1
        b_2 = 3 * (1 - t) * (t**2) * p_2
        b_3 = (t**3) * p_3

        return b_0 + b_1 + b_2 + b_3

    def angle_interpolation(self, keyframes, perception):

        target_joints = {}
        if self.start_time == -1:
            self.start_time = perception.time

        start_time = perception.time - self.start_time
        names, times, keys = keyframes

        for i in range(len(names)):
            joint = names[i]
            time = times[i]
            key = keys[i]

            if joint in self.joint_names:
                for j in range(len(time) - 1):
                    if start_time < time[0] and j == 0:
                        t_0 = 0.0
                        t_3 = time[0]
                        p_0 = perception.joint[joint]
                        p_3 = key[0][0]
                    elif time[j] < start_time < time[j+1]:
                        t_0 = time[j]
                        t_3 = time[j + 1]
                        p_0 = key[j][0]
                        p_3 = key[j + 1][0]
                    else:
                        continue
                    p_1 = key[j][1][1] + p_0
                    p_2 = key[j][2][1] + p_3
                    t = (start_time - t_0) / (t_3 - t_0)
                    target_joints[joint] = self.calculate_bezier_interpolation(p_0, p_1, p_2, p_3, t)

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  
    agent.run()
