# Author: True Price <jtprice at cs.unc.edu>

import numpy as np

#-------------------------------------------------------------------------------
#
# Image
#
#-------------------------------------------------------------------------------

class Image:
    def __init__(self, name_, camera_id_, q_, tvec_):
        self.name = name_
        self.camera_id = camera_id_
        self.q = q_
        self.tvec = tvec_

        self.points2D = np.empty((0, 2), dtype=np.float64)
        self.point3D_ids = np.empty((0,), dtype=np.uint64)

    #---------------------------------------------------------------------------

    def R(self):
        return self.q.ToR()

    #---------------------------------------------------------------------------

    def C(self):
        return -self.R().T.dot(self.tvec)

    #---------------------------------------------------------------------------

    def Pose(self):
        return  np.concatenate((np.concatenate((self.R(), np.array([self.t]).T), axis=1), np.array([[0,0,0,1]])), axis = 0)  

    def InvPose(self):
        return  np.concatenate((np.concatenate((self.R().T, np.array([self.C()]).T), axis=1), np.array([[0,0,0,1]])), axis = 0)      

    #---------------------------------------------------------------------------
    @property
    def t(self):
        return self.tvec
