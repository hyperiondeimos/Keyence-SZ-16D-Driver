#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from sz16d import KeyenceSZ16Dping
import time

def main():
    laser = KeyenceSZ16Dping()
    theta = np.arange((-45 * np.pi)/180, (225 * np.pi)/180, 1/751)

    while True:
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
        ax.plot(theta, laser)
        ax.set_rmax(2)
        ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
        ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
        ax.grid(True)

        ax.set_title("A line plot on a polar axis", va='bottom')
        plt.show()
        time.sleep(1)

if __name__=="__main__":
    main()