"""Minimal SE3 compatibility shim for ExplORB-SLAM's decision_maker.

ExplORB-SLAM's Python decision maker imports `sophus` (the Zaragoza/Lie-group
`sophus` bindings) and uses only three members:

    SE3(R, t)          rotation matrix (3x3 array) + translation
    T1 * T2            pose composition      (A*B) p == A(B p)
    T * p              point/array transform  R p + t
    T.inverse()
    T.translation()
    T.rotationMatrix()

This shim implements exactly those semantics in numpy. Convention: a pose T_ab
is the map p_a = R_ab p_b + t_ab.

NOTE: this is a dependency-adaptation shim, not a re-implementation of Sophus.
It exists because no `sophus` python binding installable without root was
available; the algebra below is the standard 3D rigid-body composition.
"""
import numpy as np


class SE3(object):
    def __init__(self, R=None, t=None):
        self.R = np.eye(3) if R is None else np.array(R, dtype=float).reshape(3, 3)
        self.t = np.zeros(3) if t is None else np.array(t, dtype=float).reshape(3)

    # --- constructors -------------------------------------------------------
    @staticmethod
    def identity():
        return SE3()

    @staticmethod
    def exp(xi):
        raise NotImplementedError("shim: only composition maths is provided")

    # --- core algebra -------------------------------------------------------
    def __mul__(self, other):
        if isinstance(other, SE3):
            return SE3(self.R.dot(other.R), self.R.dot(other.t) + self.t)
        p = np.asarray(other, dtype=float)
        if p.shape == (3,):
            return self.R.dot(p) + self.t
        if p.shape[0] == 3:                       # Nx3 stack
            return (self.R.dot(p.T) + self.t[:, None]).T
        raise ValueError("shim: unsupported operand shape %r" % (p.shape,))

    def __rmul__(self, other):
        return self.__mul__(other)

    def inverse(self):
        Rt = self.R.T
        return SE3(Rt, -Rt.dot(self.t))

    def translation(self):
        return self.t

    def rotationMatrix(self):
        return self.R

    def rotation_matrix(self):
        return self.R

    # --- dunder -------------------------------------------------------------
    def __repr__(self):
        return "SE3(R=%s, t=%s)" % (np.round(self.R, 6).tolist(), np.round(self.t, 6).tolist())

    def __eq__(self, other):
        return isinstance(other, SE3) and np.allclose(self.R, other.R) and np.allclose(self.t, other.t)
