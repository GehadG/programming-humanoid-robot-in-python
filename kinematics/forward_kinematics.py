'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np
from angle_interpolation import AngleInterpolationAgent

OFFSETS = {

}


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],

                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
                       }
        '''
            self.joint_offsets: { joint_name: [X, Y, Z, rotation around*], ... }
            *rotation around: 
                1 -> X
                2 -> Y 
                3 -> Z 
                4 -> YZ (special case: pelvis)
            
        '''
        self.joint_offsets = {
            'HeadYaw': [0.0, 0.0, 126.5, 3],
            'HeadPitch': [0.0, 0.0, 0.0, 2],
            'LShoulderPitch': [0.0, 98.0, 100.0, 2],
            'LShoulderRoll': [0.0, 0.0, 0.0, 1],
            'LElbowYaw': [105.0, 15.0, 0.0, 3],
            'LElbowRoll': [0.0, 0.0, 0.0, 1],
            'RShoulderPitch': [0.0, -98.0, 100.0, 2],
            'RShoulderRoll': [0.0, 0.0, 0.0, 1],
            'RElbowYaw': [105.0, -15.0, 0.0, 3],
            'RElbowRoll': [0.0, 0.0, 0.0, 1],
            'LHipYawPitch': [0.0, 50.0, -85.0, 4],
            'LHipRoll': [0.0, 0.0, 0.0, 1],
            'LHipPitch': [0.0, 0.0, 0.0, 2],
            'LKneePitch': [0.0, 0.0, -100.0, 2],
            'LAnklePitch': [0.0, 0.0, -102.90, 2],
            'LAnkleRoll': [0.0, 0.0, 0.0, 1],
            'RHipYawPitch': [0.0, -50.0, -85.0, 4],
            'RHipRoll': [0.0, 0.0, 0.0, 1],
            'RHipPitch': [0.0, 0.0, 0.0, 2],
            'RKneePitch': [0.0, 0.0, -100.0, 2],
            'RAnklePitch': [0.0, 0.0, -102.90, 2],
            'RAnkleRoll': [0.0, 0.0, 0.0, 1]
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        if joint_name in self.joint_offsets:

            s = np.sin(joint_angle)
            c = np.cos(joint_angle)
            R_x = matrix([[1, 0, 0, 0],
                         [0, c, -s, 0],
                         [0, s, c, 0],
                         [0, 0, 0, 1]])


            R_y = matrix([[c, 0, s, 0],
                         [0, 1, 0, 0],
                         [-s, 0, c, 0],
                         [0, 0, 0, 1]])

            R_z = matrix([[c, s, 0, 0],
                         [-s, c, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

            if self.joint_offsets[joint_name][3] == 1:
                T = R_x

            elif self.joint_offsets[joint_name][3] == 2:
                T = R_y

            elif self.joint_offsets[joint_name][3] == 3:
                T = R_z

            elif self.joint_offsets[joint_name][3] == 4:
                T = np.dot(R_y, R_z)

            T[-1,0] = self.joint_offsets[joint_name][0]
            T[-1,1] = self.joint_offsets[joint_name][1]
            T[-1,2] = self.joint_offsets[joint_name][2]
            
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)

                T = np.dot(T, Tl)
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
