import numpy as np

CLOSED_LOOP = 0


def initParameters():
    """

    This functions will create new parameters for your controller.
    [[value, min, max], ...]
    These parameters will be available in the GUI
    They will be accessible in the other functions in the params[i] list

    You must return a list with the initial values and limits you want

    For example :
    return [[1, 0, 1, "description1"], [2, -2, 4, "description2"], [3, 0, 10, "description3"]]
    will create 3 parameters where params[0] = 1 ... params[2] = 3

    Returns:
        param: list of initial value of the parameters to be updated by the GUI_ with limits
    """

    params = [[1.0, -10., 10., "kp"]] # [value, min, max, description]
    params += [[1.0, -10., 10., "ki"]]
    params += [[20.0, 0., 100., "Saturation (mm)"]]
    return params


def init_control_state(q_e, q_s, q_user, x_c_init, param, dt):
    """_summary_

    Args:
        q_e (_type_): position of the effector
        q_s (_type_): position of the sensor (real marker)
        q_user (_type_): position of the target givent by the user (GUI)
        param (_type_): list of user parameters defined in initParameters and updated by the user (GUI)
        dt (_type_): sampling period (simulation time, real time if the computer is fast enough)

    Returns:
        x_c_init: initial state of the controller
    """
    # this function is called only once and just before the first update control
    # this function should return the initial internal state of the controller
    x_c_init = np.array([0.0, -160.0, 0.0])
    return x_c_init


def update_control(q_e, q_s, q_user, x_c_prev, param, dt, ):
    """_summary_

    Args:
        q_e (_type_): position of the effector
        q_s (_type_): position of the sensor (real marker)
        q_user (_type_): position of the target givent by the user (GUI)
        x_c_prev (_type_): previous state of the controller
        param (_type_): list of user parameters defined in initParameters and updated by the user (GUI)
        dt (_type_): sampling period (simulation time, real time if the computer is fast enough)

    returns:
        q_t: position of the target to send to the inverse model
        x_c_new: new updated controller state
    """

    x_c_new = x_c_prev
    q_t = q_user
    return q_t, x_c_new
