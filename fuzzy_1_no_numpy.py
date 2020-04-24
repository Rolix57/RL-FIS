# -*- coding: utf-8 -*-
"""
Created on Tue Mar 24 17:36:13 2020

@author: broke
"""


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
    is_left_of_center = params['is_left_of_center'] 
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
    
    #object distance [0 - 1]
    # distance = 0
    # close_dist = zmf(distance, 0.1316, 0.4649)
    # far_dist = smf(distance,  0.09656, 0.958)
    
    #distance_center
    middle_d_center = zmf(distance_from_center, 0.1, 0.5)
    #center_d_center = gauss2mf(d_center, 0.1079, 0.4188, 0.1079, 0.5812)
    edge_d_center = smf(distance_from_center, 0.2, 0.9)
    
    
    ####################### RULES #########################################
    
    # x = math.arange(-1,1,0.01)
    x = list(range(-100, 100))
    for i in(range(len(x))):
        x[i] = x[i] / 100
    
    #1
    rule_1 = list()
    rule_strength = min((middle_d_center, center_steer))
    for i in x:
        rule_1.append(smf(i, -0.4, 0.5))
        if rule_1[-1] > rule_strength:
           rule_1[-1] = rule_strength
    
    #2
    rule_2 = list()
    rule_strength = min((middle_d_center, left_steer))
    for i in x:
        rule_2.append(zmf(i, -0.8, -0.02))
        if rule_2[-1] > rule_strength:
           rule_2[-1] = rule_strength
    
    
    #3
    rule_3 = list()
    rule_strength = min((middle_d_center, right_steer))
    for i in x:
        rule_3.append(zmf(i, -0.8, -0.02))
        if rule_3[-1] > rule_strength:
           rule_3[-1] = rule_strength
           
    #4
    rule_4 = list()
    rule_strength = min((edge_d_center, center_steer, slow_speed))
    for i in x:
        rule_4.append(zmf(i, -0.8, -0.02))
        if rule_4[-1] > rule_strength:
           rule_4[-1] = rule_strength
   
    #5
    rule_5 = list()
    rule_strength = min((edge_d_center, center_steer, fast_speed))
    for i in x:
        rule_5.append(zmf(i, -0.8, -0.02))
        if rule_5[-1] > rule_strength:
           rule_5[-1] = rule_strength

    #izquierda
    if is_left_of_center:
        #6i
        rule_6 = list()
        rule_strength = min((edge_d_center, left_steer))
        for i in x:
            rule_6.append(zmf(i, -0.8, -0.02))
            if rule_6[-1] > rule_strength:
               rule_6[-1] = rule_strength
       
        
        #7i
        rule_7 = list()
        rule_strength = min((edge_d_center, right_steer))
        for i in x:
            rule_7.append(smf(i, -0.4, 0.5))
            if rule_7[-1] > rule_strength:
               rule_7[-1] = rule_strength
    #derecha
    else:
        #6d
        rule_6 = list()
        rule_strength = min((edge_d_center, right_steer))
        for i in x:
            rule_6.append(zmf(i, -0.8, -0.02))
            if rule_6[-1] > rule_strength:
               rule_6[-1] = rule_strength
       
        
        #7d
        rule_7 = list()
        rule_strength = min((edge_d_center, left_steer))
        for i in x:
            rule_7.append(smf(i, -0.4, 0.5))
            if rule_7[-1] > rule_strength:
               rule_7[-1] = rule_strength
               
   ################### AGGREGATION ####################################
    aggregation = list()
    for i in range(len(rule_1)):
        aggregation.append(max(rule_1[i], rule_2[i], rule_3[i], rule_4[i],
                               rule_5[i], rule_6[i], rule_7[i]))
    
    #################### CENTROID #######################################
    
    # centroid = math.sum(math.multiply(aggregation, x)) / math.sum(aggregation)
    num = list()
    for i in range(len(aggregation)):
        num.append(aggregation[i] * x[i])
        
    centroid = sum(num) / sum(aggregation)
    
    if all_wheels_on_track:
        reward = centroid
    else:
        reward = -10.0
    
    return math.exp(reward)

print(reward_function(params))