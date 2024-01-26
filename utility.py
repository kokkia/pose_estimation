# -*- coding: utf-8 -*-
# kal便利関数ライブラリ
import math
import numpy as np

def limit_range(mini, maxi, x):
    if x>maxi:
        x=maxi
    elif x<mini:
        x = mini
    return x

def pi2pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

    
