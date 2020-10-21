import numpy as np

def coef(x0,x1,t0,t1,dx0=0,ddx0=0,dx1=0,ddx1=0):

    L=np.linalg.inv(np.array([[t0**5, t0**4, t0**3, t0**2, t0, 1],
                              [5*t0**4, 4*t0**3, 3*t0**2, 2*t0, 1, 0],
                              [20*t0**3, 12*t0**2, 6*t0, 2, 0, 0],
                              [t1 ** 5, t1 ** 4, t1 ** 3, t1 ** 2, t1, 1],
                              [5 * t1 ** 4, 4 * t1 ** 3, 3 * t1 ** 2, 2 * t1, 1, 0],
                              [20 * t1 ** 3, 12 * t1 ** 2, 6 * t1, 2, 0, 0]]))

    coefficients = L @ np.array([x0, dx0, ddx0, x1, dx1, ddx1]).T
    return coefficients

def min_jerk_zeros(points,t):
    N=t.shape[0]
    c=np.zeros((N-1,6,3))
    for i in range(N-1):
        c[i, :, 0]  = coef(points[i, 0], points[i + 1, 0], t[i], t[i + 1])
        c[i, :, 1] = coef(points[i, 1], points[i + 1, 1], t[i], t[i + 1])
        c[i, :, 2] = coef(points[i, 2], points[i + 1, 2], t[i], t[i + 1])
    return c

def desired_states(point):
    return