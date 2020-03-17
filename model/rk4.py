import numpy as np

# ------------------ RK4 STEP FUNCTION --------------------------------------------
def rk4_step(y_n, c_in, dt, fy, fv):
    k1 = np.concatenate((fy(y_n), fv(y_n, c_in)), axis=None)
    k2 = np.concatenate((fy(y_n + (dt / 2) * k1), fv(y_n + (dt / 2) * k1, c_in)), axis=None)
    k3 = np.concatenate((fy(y_n + (dt / 2) * k2), fv(y_n + (dt / 2) * k2, c_in)), axis=None)
    k4 = np.concatenate((fy(y_n + dt * k3), fv(y_n + dt * k3, c_in)), axis=None)

    return y_n + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
