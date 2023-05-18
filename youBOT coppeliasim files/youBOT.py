# -*- coding: utf-8 -*-
"""
Created on Mon Mar 13 19:29:13 2023

@author: Admin
"""

import numpy as np
import time

try:
  import sim
except:
  print ('--------------------------------------------------------------')
  print ('"sim.py" could not be imported. This means very probably that')
  print ('either "sim.py" or the remoteApi library could not be found.')
  print ('Make sure both are in the same folder as this file,')
  print ('or appropriately adjust the file "sim.py"')
  print ('--------------------------------------------------------------')
  print ('')  

client_ID = []


def sim_init():
  global sim
  global client_ID
  
  #Initialize sim interface
  sim.simxFinish(-1) # just in case, close all opened connections
  client_ID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim    
  if client_ID!=-1:
    print ('Connected to remote API server')
    return True
  else:
    return False


def sim_shutdown():
  #Gracefully shutdown simulation

  global sim
  global client_ID

  #Stop simulation
  res = sim.simxStopSimulation(client_ID, sim.simx_opmode_oneshot_wait)
  if res == sim.simx_return_ok:
    print ("---!!! Stopped Simulation !!! ---")

  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
  sim.simxGetPingTime(client_ID)

  # Now close the connection to CoppeliaSim:
  sim.simxFinish(client_ID)      
  
  return            


'''def planning():
     plan the path from start to goal
     and return the points through which it needs to go as an array.
     This arrray will be passed to go_to_goal() function, to move our robot from start to end
CAN WE DO A* and another comparative algoritm.
Maybe PRM with RRT?

'''
def add_obs():
    global sim
    global client_ID
    size = [0.1, 0.1, 0.1]
    mass = 0.1
    
    # Create the primitive shape
    shape = sim.simxCheckCollision(0, size,mass,0)
    
    # Define the position and orientation of the primitive shape
    position = [0.5, 0, 0.1]
    orientation = [0, 0, np.pi/2]
    
    # Set the position and orientation of the primitive shape
    sim.simxSetObjectPosition(client_ID, shape, -1, position, sim.simx_opmode_oneshot)
    sim.simxSetObjectOrientation(client_ID, shape, -1, orientation, sim.simx_opmode_oneshot)
    return

    
def main(path):
    # Prepare initial values and retrieve handles:
        
    global sim
    global client_ID
    global wheel_joints  # front left, rear left, rear right, front right
    if sim_init():
      wheel_joints=[-1,-1,-1,-1]
      res, wheel_joints[0] = sim.simxGetObjectHandle(client_ID, './rollingJoint_fl', sim.simx_opmode_blocking)
      res, wheel_joints[1] = sim.simxGetObjectHandle(client_ID, './rollingJoint_rl', sim.simx_opmode_blocking)
      res, wheel_joints[2] = sim.simxGetObjectHandle(client_ID, './rollingJoint_rr', sim.simx_opmode_blocking)
      res, wheel_joints[3] = sim.simxGetObjectHandle(client_ID, './rollingJoint_fr', sim.simx_opmode_blocking)
      #print("joints1:",wheel_joints[0])
      arm_joints = []
      for i in range(5):
        res, j=sim.simxGetObjectHandle(client_ID, './youBotArmJoint' + str(i), sim.simx_opmode_blocking)
        arm_joints.append(j)
        #print(arm_joints)
      


      
      for i in range(len(path)):
          res,target=sim.simxGetObjectHandle(client_ID, './Target', sim.simx_opmode_blocking)
          inter_pos = np.array(path[i])
          sim.simxSetObjectPosition(client_ID, target, -1, inter_pos, sim.simx_opmode_oneshot)
          _,goal=sim.simxGetObjectPosition(client_ID, target,-1,sim.simx_opmode_blocking)
              
          #start state
          res,start=sim.simxGetObjectHandle(client_ID, './youBot_ref', sim.simx_opmode_blocking)
    
    
          _,youBot_pos=sim.simxGetObjectPosition(client_ID, start,-1,sim.simx_opmode_blocking)
          print("Start:",youBot_pos)   
          
          #(x,y,phi) values
          #set_movement(0, 0,0)
    
          go_to_goal(youBot_pos, goal)    
      '''
      res, object1Handle = sim.simxGetObjectHandle(client_ID, './Cuboid[4]', sim.simx_opmode_blocking)
      res, youbot = sim.simxGetObjectHandle(client_ID, './ME_Platfo2_sub1', sim.simx_opmode_blocking)

      # Check for collisions between the two objects
      res, collisionState = sim.simxCheckCollision(client_ID, object1Handle, youbot , sim.simx_opmode_blocking)
      print(collisionState)
      '''
      set_movement(0, 0, 0)  
      '''
      for i in range(5):
       sim.simxSetJointPosition(client_ID, arm_joints[0],30+2*i, sim.simx_opmode_blocking)
       time.sleep(1.0)
       sim.simxSetJointPosition(client_ID, arm_joints[3],30+2*i, sim.simx_opmode_blocking)
       time.sleep(1.0)
       arm_joints = []
       for i in range(5):
         res, j=sim.simxGetObjectHandle(client_ID, './youBotArmJoint' + str(i), sim.simx_opmode_blocking)
         arm_joints.append(j)
         print(arm_joints)
    else:
        print ('Failed connecting to remote API server')
        '''
    time.sleep(5.0)
    sim_shutdown()
    time.sleep(2.0)
    return    
     
def go_to_goal(youBot_pos,goal):
    #set_movement(0, 0,0)
    #set_movement(0, 0, 0.5)
    
    dist_to_goal = np.sqrt((goal[0]-youBot_pos[0])**2 + (goal[1]-youBot_pos[1])**2)
    #print(dist_to_goal)
    speed = 0.7
    
    # move the youBot towards the goal until it reaches the goal position
    while dist_to_goal > 0.1:
        res,start=sim.simxGetObjectHandle(client_ID, './youBot_ref', sim.simx_opmode_blocking)
        _,youBot_pos=sim.simxGetObjectPosition(client_ID, start,-1,sim.simx_opmode_blocking)

        direction = np.array([youBot_pos[0]-goal[0],youBot_pos[1] -goal[1], 0])
        direction /= np.linalg.norm(direction)

        
        # set the speed of the youBot's wheels based on the direction vector
        front_back_speed = -speed *direction[1]
        right_left_speed = -speed * direction[0]          
        
        # vy,vx,phi
        set_movement(front_back_speed,right_left_speed,0)
        # calculate the distance between the current position and the goal position
        dist_to_goal = np.sqrt((goal[0]-youBot_pos[0])**2 + (goal[1]-youBot_pos[1])**2)

    # stop the youBot when it reaches the goal position
    #set_movement(0, 0,0)
    print("Reached goal position:", goal)
   

def set_movement(forw_back_vel, left_right_vel, rot_vel):
    global clientID
    # Apply the desired wheel velocities:
    sim.simxSetJointTargetVelocity(client_ID, wheel_joints[0], (-forw_back_vel - left_right_vel - rot_vel), sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_ID,wheel_joints[1], (-forw_back_vel + left_right_vel - rot_vel), sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_ID,wheel_joints[2], (-forw_back_vel - left_right_vel + rot_vel), sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_ID,wheel_joints[3], (-forw_back_vel + left_right_vel + rot_vel), sim.simx_opmode_blocking)


if __name__ == '__main__':
    #main([[0.1, 0.1, 0], [0.34, 0.34, 0], [0.68, 0.42, 0], [0.98, 0.58, 0], [1.19, 0.85, 0], [1.53, 0.92, 0], [1.87, 0.9, 0], [1.93, 0.74, 0], [1.96, 0.77, 0], [2.3, 0.75, 0], [2.64, 0.67, 0], [2.95, 0.82, 0], [3.29, 0.75, 0], [3.56, 0.53, 0], [3.89, 0.63, 0], [4.11, 0.9, 0], [4.42, 0.74, 0], [4.59, 1.04, 0], [4.64, 1.38, 0], [4.74, 1.71, 0], [4.77, 2.05, 0], [4.82, 2.39, 0], [4.84, 2.73, 0], [4.82, 3.07, 0], [4.85, 3.41, 0], [4.87, 3.75, 0], [4.83, 4.09, 0], [4.76, 4.43, 0], [5.0, 5.0, 0]])                    
    main([[0.1, 0.1, 0], [0.3733333333, 0.2866666667, 0], [0.6666666667, 0.4466666667, 0], [0.95, 0.6166666667, 0], [1.2333333333, 0.7833333333, 0], [1.53, 0.89, 0], [1.7766666667000002, 0.8533333333, 0], [1.92, 0.8033333333, 0], [2.0633333332999997, 0.7533333333000001, 0], [2.3, 0.73, 0], [2.63, 0.7466666666999999, 0], [2.96, 0.7466666666999999, 0], [3.2666666667, 0.7, 0], [3.58, 0.6366666666999999, 0], [3.8533333333, 0.6866666667, 0], [4.14, 0.7566666666999999, 0], [4.3733333333, 0.8933333333, 0], [4.55, 1.0533333333000001, 0], [4.6566666667, 1.3766666667, 0], [4.7166666667, 1.7133333332999998, 0], [4.7766666667, 2.05, 0], [4.81, 2.39, 0], [4.8266666666999996, 2.73, 0], [4.8366666667, 3.07, 0], [4.8466666667, 3.41, 0], [4.85, 3.75, 0], [4.82, 4.09, 0], [4.8633333333, 4.5066666667, 0], [5.0, 5.0, 0]])
    print ('Program ended')