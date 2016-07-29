#!/usr/bin/python

import numpy
import matplotlib.pyplot as plt
import math

def loss(x, mean, stddev):
    return 1 / (stddev + math.exp(abs(x - mean)))

mean = 0.0
stddev_x = math.sqrt(2.172)
stddev_y = math.sqrt(7.721)
num = 10000

samples_x = numpy.random.normal(mean, stddev_x, num)
samples_y = numpy.random.normal(mean, stddev_y, num)

weights_x = [loss(samples_x[i], mean, stddev_x) for i in range(num)]
weights_y = [loss(samples_y[i], mean, stddev_y) for i in range(num)]

plt.plot(samples_x, weights_x, 'ro')
plt.plot(samples_y, weights_y, 'bo')
plt.show()
