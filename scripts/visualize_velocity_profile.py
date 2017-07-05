#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from hector_nav2.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
import matplotlib.pyplot as plotter

def feedback_callback(data):
  global trajectory

  if not data: # empty
    trajectory = []
    return
  trajectory = data.trajectory
  
  
def plot_velocity_profile(fig, ax_v, ax_omega, ax_acc, t, v, omega, acc):
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('Trans. velocity [m/s]')
  ax_v.plot(t, v, '-bx')
  ax_acc.cla()
  ax_acc.grid()
  ax_acc.set_ylabel('Trans. acceleration [m/s^2]')
  ax_acc.plot(t, acc, '-bx')
  ax_omega.cla()
  ax_omega.grid()
  ax_omega.set_ylabel('Rot. velocity [rad/s]')
  ax_omega.set_xlabel('Time [s]')
  ax_omega.plot(t, omega, '-bx')
  fig.canvas.draw()

  
  
def velocity_plotter():
  global trajectory
  rospy.init_node("visualize_velocity_profile", anonymous=True)
  
  topic_name = "/hector_nav2/teb_feedback"
  #topic_name = rospy.get_param('~feedback_topic', topic_name)
  rospy.Subscriber(topic_name, TrajectoryMsg, feedback_callback, queue_size = 1) # define feedback topic here!

  rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name) 
  rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

  # two subplots sharing the same t axis
  fig, (ax_v, ax_omega, ax_acc) = plotter.subplots(3, sharex=True)
  plotter.ion()
  plotter.show()
  

  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():
    
    t = []
    acc = []
    v = []
    omega = []
    
    for point in trajectory:
      t.append(point.time_from_start.to_sec())
      v.append(point.velocity.linear.x)
      acc.append(point.acceleration.linear.x)
      omega.append(point.velocity.angular.z)
          
    plot_velocity_profile(fig, ax_v, ax_omega, ax_acc, np.asarray(t), np.asarray(v), np.asarray(omega), np.asarray(acc))
        
    r.sleep()

if __name__ == '__main__': 
  try:
    trajectory = []
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass

