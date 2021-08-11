#! /usr/bin/env python

import numpy as np


def normalize(omega):
    """
    normalize to range [-1, 1] given an angle of omega [-pi, pi]
    """
    omega = np.clip(omega, a_min=-np.pi, a_max=np.pi)
    return (omega + np.pi) / (2 * np.pi) * 2 - 1
