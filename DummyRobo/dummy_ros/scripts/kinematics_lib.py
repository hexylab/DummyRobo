from math import * 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d as a3
import numpy as np
import pathlib 
import scipy as sp
import pylab as pl
from scipy.spatial import ConvexHull
import pickle

class DummyArm:
    def __init__(self, initial_q1, initial_q2, initial_q3, initial_ee):
        self.L1 = 61
        self.L2 = 80
        self.L3 = 80
        self.L4 = 57
        self.q1 = initial_q1
        self.q2 = initial_q2
        self.q3 = initial_q3
        self.q4 = -initial_q3
        self.ee = initial_ee
        self.q1_min = -85
        self.q1_max = 85
        self.q2_min = 10
        self.q2_max = 120

    def q3CalcLimits(self, **kwargs):
        q2 = kwargs.get('q2', self.q2)
        q3_min = (-0.6755 * q2) - 70.768
        q3_max = (-0.7165 * q2) - 13.144
        return q3_min, q3_max

    def updateJointAngles(self, q1, q2, q3):
        self.checkErrorJointLimits(q1=q1, q2=q2, q3=q3)
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = -q3

    def checkErrorJointLimits(self, **kwargs):
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)
        q3_min, q3_max = self.q3CalcLimits(q2=q2)
        q3s_min = -90
        q3s_max = 30
        q3s = 90 - (180 + self.q3)
        if q1 < self.q1_min or q1 > self.q1_max:
            self.error()
            exit()
        if q2 < self.q2_min or q2 > self.q2_max:
            self.error()
            exit()
        if q3s < q3s_min or q3s > q3s_max:
            print(q3s)
            self.error()
            exit()
            
    def error(self):
        print("Error")

    def openEndEffector(self):
        self.ee = 50
        return self.ee

    def closeEndEffector(self):
        self.ee = -50
        return self.ee

    def updateEndEffector(self, ee):
        if(ee == 0):
            self.ee = -50
        elif(ee == 1):
            self.ee = 50
    
    def getJointState(self):
        q1s = self.q1
        q2s = 90 - self.q2
        q3s = 90 - (180 + self.q3)
        q4s = -q3s -q2s
        ees = self.ee
        return q1s, q2s, q3s, q4s, ees

    def getRadJointState(self):
        q1s,q2s,q3s,q4s,ees = self.getJointState()
        
        q1r = q1s * 3.14 / 180
        q2r = q2s * 3.14 / 180
        q3r = q3s * 3.14 / 180
        q4r = q4s * 3.14 / 180
        eer = ees * 3.14 / 180
        return q1r, q2r, q3r, q4r, eer


    def inverseKinematics(self, x, y, z):
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        q1 = atan2(y, x)

        x_4 = x - (L4 * cos(q1))
        y_4 = y - (L4 * sin(q1))
        z_4 = z

        z_1 = L1
        z_1_4 = z_4 - z_1
        xy_4 = sqrt((x_4**2)+(y_4**2))
        v_side = sqrt((z_1_4**2) + (xy_4**2))
        q3 = - (pi - acos((L2**2 + L3**2 - v_side**2)/(2 * L2 * L3)))
        q2_a = atan2(z_1_4, xy_4)
        q2_b = acos((v_side**2 + L2**2 - L3**2)/(2 * v_side * L2))
        q2 = q2_a + q2_b

        if(1):
            print('Input Position of End Effector: \n')
            print('x_EE: {}'.format(x))
            print('y_EE: {}'.format(y))
            print('z_EE: {} \n'.format(z))
            
            print('Ouput joint angles: \n')
            print('q1: {:+.2f}'.format(q1 * 180/pi))
            print('q2: {:+.2f}'.format(q2 * 180/pi))
            print('q3: {:+.2f} \n'.format(q3 * 180/pi))

        q1 = round(q1 * 180/pi, 2)
        q2 = round(q2 * 180/pi, 2)
        q3 = round(q3 * 180/pi, 2)

        return q1, q2, q3
