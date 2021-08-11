#! /usr/bin/env python
import numpy as np


def OR(x, y):
    return x + y - x * y * np.tanh(x + y) / np.tanh(2)


def AND(x, y):
    return x * y * np.tanh(x + y) / np.tanh(2)


def COMPARE(x, y):
    return OR(x, -y)


def INVOKE(x, y):
    return AND(x, OR(x, y))


def PREVAIL(x, y):
    return OR(x, OR(x, y))


def XOR(x, y):
    return COMPARE(x, y) * COMPARE(x, y) * OR(x, y)


def NOT(x):
    if x == 0:
        x = 1
    else:
        x = np.sign(x) * (1 - np.abs(x))
    return x


def Valve(*args):
    return np.prod(args)

