#!/usr/bin/env python3

# import matplotlib
# matplotlib.use('Qt5Cairo')

"""
Backends:
['GTK3Agg', 'GTK3Cairo', 'GTK4Agg', 'GTK4Cairo', 'MacOSX', 
'nbAgg', 'QtAgg', 'QtCairo', 'Qt5Agg', 'Qt5Cairo', 'TkAgg', 
'TkCairo', 'WebAgg', 'WX', 'WXAgg', 'WXCairo', 'agg', 'cairo', 
'pdf', 'pgf', 'ps', 'svg', 'template']
"""

import matplotlib.pyplot as plt
import numpy as np
from sz16d import KeyenceSZ16Dping
import time

def main():
    laser = KeyenceSZ16Dping()
    intensities = list(laser.tick())
    theta = np.arange((-45 * np.pi)/180, (225 * np.pi)/180, ((270 * np.pi)/180)/len(intensities))

    plt.ion()
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    line1, = ax.plot(theta, intensities)
    ax.set_rmax(10000)
    ax.set_rticks([2000, 4000, 6000, 8000, 10000])  # Less radial ticks
    ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
    ax.grid(True)

    ax.set_title("Keyence SZ-16D Laser Scan", va='bottom')

    plt.show()

    while True:
        try:
            plt.cla() # clear axis
            intensities = list(laser.tick()) 
            ax.plot(theta, intensities) # Update the data
            plt.draw() 
            plt.pause(1e-17)
            time.sleep(0.01)
        except KeyboardInterrupt:
            plt.close('all')
            break

if __name__=="__main__":
    main()