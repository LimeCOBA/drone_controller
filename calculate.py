import math


def clamp(value, low, high):
    if value < low:
        return low
    else:
        if value > high:
            return high
        else:
            return value
