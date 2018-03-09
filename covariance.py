#!/usr/bin/python
# 
# 
# Make sure you have NumPy and MatplotLib installed before you run this.
# 
# This is a simulator to computationally discover the covariance matrix for 
#  a two-variable system. It uses a simple normal distribution for both variables.
# 
# This technique was inspired by this blog post:
# http://rebcabin.github.io/blog/2013/01/22/covariance-matrices/

from math import *
import numpy as np
import numpy.linalg as npl
import matplotlib.pyplot as plt
from time import sleep

# Set these values to whatever you want!
# X_M := mean of X. (the value the X distribution centers around)
# X_SD := standard deviation of X. (increases the spread of X around the mean if increased)
X_M  = 0.0
X_SD = 50.0
Y_M  = 0.0
Y_SD = 30.0

# The number of values in the "window" of stored values
# (e.g. there will be 100 most recent values stored if LEN=100)
LEN = 100

# The number of trials the code goes through
TRIALS = 1000

# The additional delay after every new point (in seconds)
DELAY = 0.05




if __name__ == '__main__':
    print "\nWelcome to Josh's running covariance simulator!"
    print "This program will calculate and depict the covariance matrix of\n\
(x,y) ~ (N(%.2f, %.2f), N(%.2f, %.2f))." % (X_M, X_SD, Y_M, Y_SD)
    print "\nSetting up matrices..."

    # Set up MatPlotLib...
    plt.ion()

    # matrix that contains all the values (just set everything to M and see what happens)
    valmat = np.transpose(np.array([ [X_M]*LEN, [Y_M]*LEN ]))
    # residual matrix (the value matrix, but every value is minus the mean)
    residual = np.transpose(np.array([ [X_M]*LEN, [Y_M]*LEN ]))
    # covariance matrix. index (0,0) is X's vairance, (0,1) and (1,0) is X and Y correlation,
    #  and (1,1) is Y's variance
    covariance = np.array([ [0, 0], [0, 0] ])

    x_mean  = 0
    y_mean  = 0
    x_var   = 0
    y_var   = 0
    x_stdev = 0
    y_stdev = 0

    sleep(5)

    for i in range(TRIALS):
        # generate random normal distribution
        xsamp = np.random.normal(X_M, X_SD)
        ysamp = np.random.normal(Y_M, Y_SD)

        # insert new value into the array and pop the last one off the list
        outxsamp, outysamp = valmat[0]
        #print np.delete(valmat, 0, 0)
        valmat = np.append(np.delete(valmat, 0, axis=0), [ [xsamp, ysamp] ], axis=0)
        #valmat = np.append(valmat, [xsamp, ysamp], axis=0)

        # calculate new x and y means
        x_mean = x_mean - outxsamp/LEN + xsamp/LEN
        y_mean = y_mean - outysamp/LEN + ysamp/LEN
        x_mean_diff = xsamp/LEN - outxsamp/LEN
        y_mean_diff = ysamp/LEN - outysamp/LEN

        # compute new residual
        for j in residual:
            j[0] = j[0] + x_mean_diff
            j[1] = j[1] + y_mean_diff

        # compute covariance matrix (yay!)
        covariance = np.array(np.matrix(np.transpose(residual)) * np.matrix(residual) / (LEN+1))

        # from the covariance matrix, determine running values in each axis
        x_var   = covariance[0][0]
        y_var   = covariance[1][1]
        x_stdev = np.sqrt(x_var)
        y_stdev = np.sqrt(y_var)

        print "\nFor a point (x,y) = (%.2f, %.2f), " % (xsamp, ysamp)
        print "the new average is (%.2f, %.2f), " % (x_mean, y_mean)
        print "and the new covariance matrix is "
        print covariance

        # clear current graph and plot all points and deviations
        plt.gcf().clear()
        plt.axis([X_M - 2.5*X_SD, X_M + 2.5*X_SD, Y_M - 2.5*Y_SD, Y_M + 2.5*Y_SD])
        plt.plot([X_M + X_SD, X_M - X_SD], [Y_M, Y_M])
        plt.plot([X_M, X_M], [Y_M + Y_SD, Y_M - Y_SD])
        plt.plot([x_mean + x_stdev, x_mean - x_stdev], [y_mean, y_mean])
        plt.plot([x_mean, x_mean], [y_mean + y_stdev, y_mean - y_stdev])
        
        plt.scatter(np.transpose(valmat)[0], np.transpose(valmat)[1])
        plt.pause(DELAY)

    while True:
        # wait until window is closed by the user, then terminate
        plt.pause(DELAY)
