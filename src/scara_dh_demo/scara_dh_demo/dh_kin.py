import numpy as np
def dh(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0.0,   sa,     ca,    d],
                     [0.0,  0.0,    0.0,  1.0]])
class SCARAKinematics:
    def __init__(self, a1=0.30, a2=0.25, d1_offset=0.10, z_sign=-1.0):
        self.a1, self.a2 = a1, a2
        self.d1_offset = d1_offset
        self.z_sign = z_sign
    def fk(self, q):
        th1, th2, d3, th4 = q
        T1 = dh(0.0, 0.0, self.d1_offset, th1)
        T2 = dh(self.a1, 0.0, 0.0, th2)
        T3 = dh(self.a2, 0.0, self.z_sign*d3, 0.0)
        T4 = dh(0.0, 0.0, 0.0, th4)
        T  = T1 @ T2 @ T3 @ T4
        pos = T[:3, 3]
        yaw = np.arctan2(T[1,0], T[0,0])
        return pos, yaw, T
    def ik(self, x, y, z, phi, elbow='up'):
        r2 = x*x + y*y
        c2 = (r2 - self.a1*self.a1 - self.a2*self.a2)/(2*self.a1*self.a2)
        c2 = np.clip(c2, -1.0, 1.0)
        s2 = np.sqrt(1 - c2*c2)
        if elbow == 'down':
            s2 = -s2
        th2 = np.arctan2(s2, c2)
        th1 = np.arctan2(y, x) - np.arctan2(self.a2*np.sin(th2), self.a1 + self.a2*np.cos(th2))
        d3  = self.z_sign * (z - 0.0)
        th4 = phi - (th1 + th2)
        return np.array([th1, th2, d3, th4])
