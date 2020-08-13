# -*- coding: utf-8 -*-

params = {'steering_angle': -24, 'speed': 0.5, 'all_wheels_on_track': True,
          'is_left_of_center': True, 'track_width': 0.4, 
          'distance_from_center': 0.19, 'objects_distance':[1.12,1.13],
          'closest_objects': [0, 1], 'objects_left_of_center':[True, False]}

import math

def zmf(x, a, b):
    value = 0
    if x <= a:
        value = 1
    elif a < x and x < (a+b)/2:
        value = 1 - 2 * ((x-a)/(b-a))**2
    elif (a+b)/2 <= x and x < b:
        value = 2 * ((x-b)/(b-a))**2
    else:
        value = 0
    return value

def smf(x, a, b):
    value = 0
    if x >= b:
        value = 1
    elif a < x and x < (a+b)/2:
        value = 2 * ((x-a)/(b-a))**2
    elif (a+b)/2 <= x and x < b:
        value = 1 - 2 * ((x-b)/(b-a))**2
    else:
        value = 0
    return value

def gaussmf(x, sigma1, c1):
    value = math.exp((-1*(x-c1)**2)/((2*sigma1)**2)) 
    return value

def gauss2mf(x, sigma1, c1, sigma2, c2):
    value = 0
    if x < c1:
       value = math.exp((-1*(x-c1)**2)/((2*sigma1)**2)) 
    elif x > c2:
        value = math.exp((-1*(x-c2)**2)/((2*sigma2)**2)) 
    else:
        value = 1
    return value


def reward_function(params):
    
    MAX_SPEED = 4
    MAX_ANGLE = 30
        
    # car booleans
    all_wheels_on_track = params['all_wheels_on_track']
    is_left_of_center = int(params['is_left_of_center'])
    steering = params['steering_angle'] / MAX_ANGLE
    speed = params['speed'] / MAX_SPEED
    track_width = params['track_width'] / 2 
    distance_from_center = params['distance_from_center'] / track_width
    
######################      INPUTS      ################################
    #speed mfs [0 - 1] 
    slow_speed = zmf(speed, 0.04167, 0.375)
    fast_speed = smf(speed, 0.204, 0.8108)
    
    #steering mfs [-1 - 1]
    left_steer = zmf(steering, -0.9167, -0.25)
    center_steer = gauss2mf(steering, 0.2831, -0.08333, 0.2831, 0.08333)
    right_steer = smf(steering, 0.25, 0.9167)
    
    #distance_center
    middle_d_center = zmf(distance_from_center, 0.05, 0.35)
    center_d_center = gaussmf(distance_from_center, 0.156, 0.4)
    edge_d_center = smf(distance_from_center, 0.5, 0.8)
    
    
    ####################### RULES #########################################
    
    # x = math.arange(-1,1,0.01)
    x = list(range(-100, 100))
    for i in(range(len(x))):
        x[i] = x[i] / 100
    
    #1
    rule_1, rule_2, rule_3, rule_4 = list(), list(), list(), list()
    rule_5, rule_6, rule_7, rule_8 = list(), list(), list(), list()
    rule_9, rule_10, rule_11 = list(), list(), list()
    
    rule_strength_1 = min((middle_d_center, center_steer))
    rule_strength_2 = min((middle_d_center, left_steer))
    rule_strength_3 = min((middle_d_center, right_steer))
    rule_strength_4 = min((center_d_center, center_steer))
    rule_strength_5 = min((edge_d_center, center_steer))
    rule_strength_6 = min((center_d_center, (is_left_of_center*left_steer) + 
                         ((1 - is_left_of_center)*right_steer)))
    rule_strength_7 = min((slow_speed, center_d_center, 
                         (is_left_of_center*right_steer) + 
                         ((1 - is_left_of_center)*left_steer)))
    rule_strength_8 = min((fast_speed, center_d_center, 
                     (is_left_of_center*right_steer) + 
                     ((1 - is_left_of_center)*left_steer)))
    rule_strength_9 = min((edge_d_center, 
                         (is_left_of_center*left_steer) + 
                         ((1 - is_left_of_center)*right_steer)))
    rule_strength_10 = min((slow_speed, edge_d_center, 
                         (is_left_of_center*right_steer) + 
                         ((1 - is_left_of_center)*left_steer)))
    rule_strength_11 = min((fast_speed, edge_d_center, 
                         (is_left_of_center*right_steer) + 
                         ((1 - is_left_of_center)*left_steer)))
    
    ####################### EVALUATION ##################################
    for i in x:
        # rule 1
        rule_1.append(rule_strength_1 
                      if smf(i, 0.0238, 0.938) > rule_strength_1 
                      else smf(i, 0.0238, 0.938))
           
       # rule 2
        rule_2.append(rule_strength_2 
                      if gaussmf(i, 0.2224, 0.0) > rule_strength_2 
                      else gaussmf(i, 0.2224, 0.0))

       # rule 3
        rule_3.append(rule_strength_3 
                      if gaussmf(i, 0.2224, 0.0) > rule_strength_3 
                      else gaussmf(i, 0.2224, 0.0))

       # rule 4
        rule_4.append(rule_strength_4 
                      if gaussmf(i, 0.2224, 0.0) > rule_strength_4 
                      else gaussmf(i, 0.2224, 0.0))

        # rule 5
        rule_5.append(rule_strength_5
                      if zmf(i, -0.938, -0.02381) > rule_strength_5
                      else zmf(i, -0.938, -0.02381))

        # rule 6
        rule_6.append(rule_strength_6
                      if zmf(i, -0.938, -0.02381) > rule_strength_6
                      else zmf(i, -0.938, -0.02381))
           
       # rule 7
        rule_7.append(rule_strength_7 
                      if smf(i, 0.0238, 0.938) > rule_strength_7 
                      else smf(i, 0.0238, 0.938))
           
       # rule 8
        rule_8.append(rule_strength_8
                      if gaussmf(i, 0.2224, 0.0) > rule_strength_8
                          else gaussmf(i, 0.2224, 0.0))

        # rule 9
        rule_9.append(rule_strength_9
                      if zmf(i, -0.938, -0.02381) > rule_strength_9
                      else zmf(i, -0.938, -0.02381))

        # rule 10
        rule_10.append(rule_strength_10
                       if gaussmf(i, 0.2224, 0.0) > rule_strength_10 
                       else gaussmf(i, 0.2224, 0.0))
           
        # rule 11
        rule_11.append(rule_strength_11
                       if smf(i, 0.0238, 0.938) > rule_strength_11
                       else smf(i, 0.0238, 0.938))
              
   ################### AGGREGATION ####################################
    aggregation = list()
    for i in range(len(rule_1)):
        aggregation.append(max(rule_1[i], rule_2[i], rule_3[i], rule_4[i],
                               rule_5[i], rule_6[i], rule_7[i], rule_8[i],
                               rule_9[i], rule_10[i], rule_11[i]))
    
    #################### CENTROID #######################################
    
    num = list()
    for i in range(len(aggregation)):
        num.append(aggregation[i] * x[i])
        
    centroid = sum(num) / sum(aggregation)
    
    if all_wheels_on_track:
        reward = centroid
    else:
        reward = -2.0
    
    return math.exp(reward)

print(reward_function(params))