#!/usr/bin/env python

import rospy
import numpy as np
from threading import Lock

'''
  Provides methods for re-sampling from a distribution represented by weighted samples
'''
class ReSampler:

  '''
    Initializes the resampler
    particles: The particles to sample from
    weights: The weights of each particle
    state_lock: Controls access to particles and weights
  '''
  def __init__(self, particles, weights, state_lock=None):
    self.particles = particles
    self.weights = weights

    # For speed purposes, you may wish to add additional member variable(s) that
    # cache computations that will be reused in the re-sampling functions
    # YOUR CODE HERE?

    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock

  '''
    Performs independently, identically distributed in-place sampling of particles
  '''
  def resample_naiive(self):
    self.state_lock.acquire()

    #get local particles and weights
    particles[:] = self.particles
    weights[:] = self.weights

    #create new N_PARTICLES bins
    new_particles = []

    weights_norm = sum(weights)
    for i in xrange(len(particles)):
        weights[i] = (weights[i]/weights_norm)
    #print sum(weights)

    #draw between 0-1 randomly and keep the particles of the bin that it falls in
    for i in xrange(len(particles)):
        rand_int = np.random.choice(1000)
        rand = rand_int * 0.001

        #find particle of weight bin
        index = 0
        weight_sum = 0
        #print rand
        while (rand > (weights[index] + weight_sum)):
            weight_sum += weights[index]
            index += 1
            #print rand
            #print weights[index] + weight_sum

        #print index

        #append particle to new_particles
        new_particles.append(particles[index])

    #update particles in line
    self.particles[:] = new_particles[:]

    # YOUR CODE HERE

    self.state_lock.release()

  '''
    Performs in-place, lower variance sampling of particles
    (As discussed on pg 110 of Probabilistic Robotics)
  '''
  def resample_low_variance(self):
    self.state_lock.acquire()

    # YOUR CODE HERE

    self.state_lock.release()

import matplotlib.pyplot as plt

if __name__ == '__main__':

  rospy.init_node("sensor_model", anonymous=True) # Initialize the node

  n_particles = int(rospy.get_param("~n_particles",100)) # The number of particles
  k_val = int(rospy.get_param("~k_val", 80)) # Number of particles that have non-zero weight
  resample_type = rospy.get_param("~resample_type", "naiive") # Whether to use naiive or low variance sampling
  trials = int(rospy.get_param("~trials", 10)) # The number of re-samplings to do

  histogram = np.zeros(n_particles, dtype=np.float) # Keeps track of how many times
                                                    # each particle has been sampled
                                                    # across trials

  for i in xrange(trials):
    particles = np.repeat(np.arange(n_particles)[:,np.newaxis],3, axis=1) # Create a set of particles
                                                                          # Here their value encodes their index
    # Have increasing weights up until index k_val
    weights = np.arange(n_particles, dtype=np.float)
    weights[k_val:] = 0.0
    weights[:] = weights[:] / np.sum(weights)

    rs = ReSampler(particles, weights) # Create the Resampler

    # Resample
    if resample_type == "naiive":
      rs.resample_naiive()
    elif resample_type == "low_variance":
      rs.resample_low_variance()
    else:
      print "Unrecognized resampling method: "+ resample_type

    # Add the number times each particle was sampled
    for j in xrange(particles.shape[0]):
      histogram[particles[j,0]] = histogram[particles[j,0]] + 1

  # Display as histogram
  plt.bar(np.arange(n_particles), histogram)
  plt.xlabel('Particle Idx')
  plt.ylabel('# Of Times Sampled')
  plt.show()
