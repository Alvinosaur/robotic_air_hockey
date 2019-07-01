#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv


class Pose2D:
    """[summary]

    """
    STATE_DIM = 4  # Dimension of state vector, [x, y, vx, vy]
    MEAS_DIM = 4   # Dimension of measurement vector
    def __init__(self, name, x, y):
        self.name = name
        self.X = np.array([[x], [y], [0], [0]])  # state vector
        self.P = np.identity(Pose2D.STATE_DIM)
        self.H = np.identity(Pose2D.MEAS_DIM)
        self.V = np.identity(Pose2D.STATE_DIM)  # process noise
        self.W = np.identity(Pose2D.MEAS_DIM)   # measurement noise

    def getX(self): return self.X[0][0]
    def getY(self): return self.X[1][0]
    def getVx(self): return self.X[2][0]
    def getVy(self): return self.X[3][0]

    def predict(self, dt):
        """[summary]
        
        Arguments:
            dt {[type]} -- [description]
        """
        A = Pose2D.generate_A(dt)
        self.X = A.dot(self.X)
        self.P = A.dot(self.P.dot(A.T)) + self.V

    def update(self, Z):
        """[summary]
        
        Arguments:
            Z {np.array} -- vector of dimension, MEAS_DIM
                         -- or None if no object detected
        """
        if Z is not None:  # can't use != since comparing np elementwise
            S = self.H.dot(self.P.dot(self.H.T)) + self.W
            K = self.P.dot(self.H.T).dot(inv(S))
            innov = Z - self.H.dot(self.X)

            self.X += K.dot(innov)
            self.P -= K.dot(self.H.dot(self.P))

    def estimate_state(self, Z, dt, V=None, W=None):
        self.predict(dt)
        self.update(Z)
        return self.X

    def __repr__(self):
        return "%s(x=%.2f, y=%.2f)" % (self.name, self.x, self.y)
    
    @staticmethod
    def generate_A(dt):
        """[summary]
        
        Arguments:
            dt {[type]} -- [description]
        
        Returns:
            [type] -- [description]
        """
        # A = array([[1, 0, dt, 0]
        #            [0, 1, 0, dt],
        #            [0, 0, 1, 0],
        #            [0, 0, 0, 1]])
        A = np.identity(Pose2D.STATE_DIM)
        A[0][2] = dt
        A[1][3] = dt
        return A
