# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        m = np.identity(params.dim_state) # (px,py,pz, vx,vy,vz)
        m[0,3] = params.dt
        m[1,4] = params.dt
        m[2,5] = params.dt
        return np.matrix(m)
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############

        Q = np.zeros((params.dim_state, params.dim_state))
        q = params.q
        dt = params.dt
        q1 = q * (dt**4) / 4
        q2 = q * (dt**3) / 3
        q3 = q * (dt**2) / 2
        q4 = q * dt
        
        Q[0,1] = Q[1,2] = Q[2,3] = q1
        Q[0,3] = Q[1,4] = Q[2,5] = q2
        Q[3,0] = Q[4,1] = Q[4,2] = q3
        Q[3,3] = Q[4,4] = Q[5,5] = q4
        
        return np.matrix(Q)
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        x = F * track.x
        P = F * track.P * F.transpose() + self.Q()
    
        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x) # measurement matrix
        gamma = self.gamma(track, meas) # residual
        P = track.P
        S = self.S(track, meas, H)
        K = P * H.transpose() * np.linalg.inv(S) # kalman gain
        x = track.x + K * gamma # state update
        
        I = np.identity(params.dim_state)
        P = (I - K * H) * P # covariance update
        
        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        hx = meas.sensor.get_hx(track.x) 
        
        return meas.z - hx 
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        S = H * track.P * H.transpose() + meas.R
        return S
        
        ############
        # END student code
        ############ 