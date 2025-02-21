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
        d = params.dt
        return np.matrix([
            [1,0,0,d,0,0],
            [0,1,0,0,d,0],
            [0,0,1,0,0,d],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1],
        ])

        ############
        # END student code
        ############

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        dt1 = params.dt
        dt2 = dt1 * dt1 / 2
        dt3 = dt1 * dt1 * dt1 / 3
        return np.matrix([
            [dt3,0  ,0  ,dt2,0  ,0  ],
            [0  ,dt3,0  ,0  ,dt2,0  ],
            [0  ,0  ,dt3,0  ,0  ,dt2],
            [dt2,0  ,0  ,dt1,0  ,0  ],
            [0  ,dt2,0  ,0  ,dt1,  0],
            [0  ,0  ,dt2,0  ,0  ,dt1],
        ]) * params.q

        ############
        # END student code
        ############

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        track.set_x(self.F() * track.x)
        track.set_P(self.F() * track.P * self.F().T + self.Q())

        ############
        # END student code
        ############

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)
        K = track.P * H.T * np.linalg.inv(S)
        x = track.x + K * gamma
        P = (np.identity(params.dim_state) - K * H) * track.P

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

        return meas.z - meas.sensor.get_hx(track.x)

        ############
        # END student code
        ############

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return H * track.P * H.T + meas.R

        ############
        # END student code
        ############