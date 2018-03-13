#script for testing a priori sensor estimation

import numpy as np
import math as math

BOX_WIDTH = 60
BOX_LENGTH = 100

def apriori_sensor_estimate(state_apriori):
    #apriori state values
    x_apriori = state_apriori[0]
    y_apriori = state_apriori[1]
    xy_apriori = [x_apriori, y_apriori]
    theta_apriori = state_apriori[2]
    
    #want to determine this value
    front_sensor_distance = 0
    right_sensor_distance = 0
    
    #special cases: theta_apriori = 0, 90, 180, or 270 deg
    if theta_apriori == 0:
        front_sensor_distance = BOX_LENGTH - x_apriori
        right_sensor_distance = y_apriori
        return front_sensor_distance , right_sensor_distance
    
    if theta_apriori == math.pi/2:
        front_sensor_distance = BOX_WIDTH - y_apriori
        right_sensor_distance = y_apriori        
        return front_sensor_distance , right_sensor_distance
    
    if theta_apriori == math.pi:
        front_sensor_distance = x_apriori
        right_sensor_distance = y_apriori        
        return front_sensor_distance , right_sensor_distance
    
    if theta_apriori == 3/2*math.pi:
        front_sensor_distance = y_apriori
        right_sensor_distance = y_apriori        
        return front_sensor_distance , right_sensor_distance
    
    #slopes
    m1 = math.tan(theta_apriori)
    m2 = -1/m1
    
    #direction vectors
    direction_vector_front = [math.cos(theta_apriori), math.sin(theta_apriori)]
    direction_vector_right = [math.sin(theta_apriori), -math.cos(theta_apriori)]
        
    #boundary intersections
    x_leftwall = 0
    y_leftwall_front = get_y_value(x_apriori, y_apriori, x_leftwall, m1)
    leftwall_intersection_front = [x_leftwall, y_leftwall_front]
    y_leftwall_right = get_y_value(x_apriori, y_apriori, x_leftwall, m2)
    leftwall_intersection_right = [x_leftwall, y_leftwall_right]

    x_rightwall = BOX_LENGTH
    y_rightwall_front = get_y_value(x_apriori, y_apriori, x_rightwall, m1)
    rightwall_intersection_front = [x_rightwall, y_rightwall_front]
    y_rightwall_right = get_y_value(x_apriori, y_apriori, x_rightwall, m2)
    rightwall_intersection_right = [x_rightwall, y_rightwall_right]    

    y_bottomwall = 0
    x_bottomwall_front = get_x_value(x_apriori, y_apriori, y_bottomwall, m1)
    bottomwall_intersection_front = [x_bottomwall_front, y_bottomwall]
    x_bottomwall_right = get_x_value(x_apriori, y_apriori, y_bottomwall, m2)
    bottomwall_intersection_right = [x_bottomwall_right, y_bottomwall]
    
    y_topwall = BOX_WIDTH
    x_topwall_front = get_x_value(x_apriori, y_apriori, y_topwall, m1)
    topwall_intersection_front = [x_topwall_front, y_topwall]
    x_topwall_right = get_x_value(x_apriori, y_apriori, y_topwall, m2)
    topwall_intersection_right = [x_topwall_right, y_topwall]
    
    #boundary vectors
    leftwall_vector_front = list(np.array(leftwall_intersection_front) - np.array(xy_apriori)) 
    rightwall_vector_front = list(np.array(rightwall_intersection_front) - np.array(xy_apriori))
    bottomwall_vector_front = list(np.array(bottomwall_intersection_front) - np.array(xy_apriori))
    topwall_vector_front = list(np.array(topwall_intersection_front) - np.array(xy_apriori))
    
    leftwall_vector_right = list(np.array(leftwall_intersection_right) - np.array(xy_apriori)) 
    rightwall_vector_right = list(np.array(rightwall_intersection_right) - np.array(xy_apriori))
    bottomwall_vector_right = list(np.array(bottomwall_intersection_right) - np.array(xy_apriori))
    topwall_vector_right = list(np.array(topwall_intersection_right) - np.array(xy_apriori))   
    
    #vector dot products
    leftwall_dotproduct_front = np.dot(direction_vector_front, leftwall_vector_front)
    rightwall_dotproduct_front = np.dot(direction_vector_front, rightwall_vector_front)
    bottomwall_dotproduct_front = np.dot(direction_vector_front, bottomwall_vector_front)
    topwall_dotproduct_front = np.dot(direction_vector_front, topwall_vector_front)
    dot_products_front = [leftwall_dotproduct_front, rightwall_dotproduct_front, bottomwall_dotproduct_front, topwall_dotproduct_front]
    
    leftwall_dotproduct_right = np.dot(direction_vector_right, leftwall_vector_right)
    rightwall_dotproduct_right = np.dot(direction_vector_right, rightwall_vector_right)
    bottomwall_dotproduct_right = np.dot(direction_vector_right, bottomwall_vector_right)
    topwall_dotproduct_right = np.dot(direction_vector_right, topwall_vector_right)
    dot_products_right = [leftwall_dotproduct_right, rightwall_dotproduct_right, bottomwall_dotproduct_right, topwall_dotproduct_right]     

    #distances between apriori (x,y) and boundary intersections
    leftwall_distance_front = dist(xy_apriori, leftwall_intersection_front)
    rightwall_distance_front = dist(xy_apriori, rightwall_intersection_front)
    bottomwall_distance_front = dist(xy_apriori, bottomwall_intersection_front)
    topwall_distance_front = dist(xy_apriori, topwall_intersection_front)
    
    leftwall_distance_right = dist(xy_apriori, leftwall_intersection_right)
    rightwall_distance_right = dist(xy_apriori, rightwall_intersection_right)
    bottomwall_distance_right = dist(xy_apriori, bottomwall_intersection_right)
    topwall_distance_right = dist(xy_apriori, topwall_intersection_right)
    
    #determine which wall front sensor is pointing at    
    positive_dot_products_front = []
    for i in range(len(dot_products_front)):
        if dot_products_front[i] > 0:                              #store the positive dot products
            positive_dot_products_front.append(dot_products_front[i])
            
    print "positive dot products for front sensor: ",positive_dot_products_front
    print " "
    
    #left wall dot product check
    if dot_products_front[0] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[0] is the smallest dot product
        for i in range(len(positive_dot_products_front)):
            if dot_products_front[0] > positive_dot_products_front[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            print "enter left"
            front_sensor_distance = leftwall_distance_front;

    #right wall dot product check
    if dot_products_front[1] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[1] is the smallest dot product
        for i in range(len(positive_dot_products_front)):
            if dot_products_front[1] > positive_dot_products_front[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            print "enter right"
            front_sensor_distance = rightwall_distance_front;
        
    #bottom wall dot product check
    if dot_products_front[2] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[2] is the smallest dot product        
        for i in range(len(positive_dot_products_front)):
            if dot_products_front[2] > positive_dot_products_front[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            print "enter bottom"
            front_sensor_distance = bottomwall_distance_front;
         
    #top wall dot product check
    if dot_products_front[3] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[3] is the smallest dot product
        for i in range(len(positive_dot_products_front)):
            if dot_products_front[3] > positive_dot_products_front[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            print "enter top"
            front_sensor_distance = topwall_distance_front;
        
    #determine which wall right sensor is pointing at    
    positive_dot_products_right = []
    for i in range(len(dot_products_right)):
        if dot_products_right[i] > 0:                              #store the positive dot products
            positive_dot_products_right.append(dot_products_right[i])
            
    print "positive dot products for right sensor: ",positive_dot_products_front
    print " "
    
    #left wall dot product check
    if dot_products_right[0] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[0] is the smallest dot product
        for i in range(len(positive_dot_products_right)):
            if dot_products_right[0] > positive_dot_products_right[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            print "enter left"
            right_sensor_distance = leftwall_distance_right;

    #right wall dot product check
    if dot_products_right[1] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[1] is the smallest dot product
        for i in range(len(positive_dot_products_right)):
            if dot_products_right[1] > positive_dot_products_right[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            print "enter right"
            right_sensor_distance = rightwall_distance_right;
        
    #bottom wall dot product check
    if dot_products_right[2] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[2] is the smallest dot product        
        for i in range(len(positive_dot_products_right)):
            if dot_products_right[2] > positive_dot_products_right[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            print "enter bottom"
            right_sensor_distance = bottomwall_distance_right;
         
    #top wall dot product check
    if dot_products_right[3] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[3] is the smallest dot product
        for i in range(len(positive_dot_products_right)):
            if dot_products_right[3] > positive_dot_products_right[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            print "enter top"
            right_sensor_distance = topwall_distance_right;
 
    #print wall intersection distances 
    print " "
    print "left wall distance front sensor: ", leftwall_distance_front
    print "right wall distance front sensor: ", rightwall_distance_front
    print "bottom wall distance front sensor: ", bottomwall_distance_front
    print "top wall distance front sensor: ", topwall_distance_front
    print " "
    print "left wall distance right sensor: ", leftwall_distance_right
    print "right wall distance right sensor: ", rightwall_distance_right
    print "bottom wall distance right sensor: ", bottomwall_distance_right
    print "top wall distance right sensor: ", topwall_distance_right
    
    #print wall dot products
    print " "
    print "left wall dot product front sensor: ", leftwall_dotproduct_front
    print "right wall dot product front sensor: ", rightwall_dotproduct_front
    print "bottom wall dot product front: ", bottomwall_dotproduct_front
    print "top wall dot product front: ", topwall_dotproduct_front
    print " "
    print "left wall dot product right sensor: ", leftwall_dotproduct_right
    print "right wall dot product right sensor: ", rightwall_dotproduct_right
    print "bottom wall dot product right: ", bottomwall_dotproduct_right
    print "top wall dot product right: ", topwall_dotproduct_right
    print " "
    
    return front_sensor_distance, right_sensor_distance

    
    
    
    
def get_y_value(x_apriori, y_apriori, x ,slope):
    return slope*(x - x_apriori) + y_apriori

def get_x_value(x_apriori, y_apriori, y, slope):
    return ((y - y_apriori) / slope) + x_apriori

def dist(p1,p2):
    return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
    
    
        
         
if __name__ == "__main__":
    
    x_apriori = input("A priori x: ")
    y_apriori = input("A priori y: ")
    theta_apriori = input("A priori theta (in radians): ")
    print " "
    
    state_apriori = [x_apriori, y_apriori, theta_apriori]
    
    aprioriFrontSensorEstimate, aprioriRightSensorEstimate = apriori_sensor_estimate(state_apriori)
    
    print "A priori front sensor estimate: ", aprioriFrontSensorEstimate 
    print "A priori right sensor estimate: ", aprioriRightSensorEstimate     
    
