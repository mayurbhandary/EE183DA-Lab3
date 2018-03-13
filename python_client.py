import numpy as np
import math as math
import websocket
import time
try:
    import thread
except ImportError:
    import _thread as thread
import time
connected=False
state_est=[28.75,19.05,0]
BOX_WIDTH = 38.1
BOX_LENGTH = 57.5
angle=0
P=[.002,.002,.002] #initial process covariance matrix diagonal elements

def on_message(ws, message):
    R=[.02478,0.7965,0.0057]#sensor noise covariance matrix diagonal elements
    global state_est
    global P
    #print(message)
    S=message.split(' ')
    paperbot_message=map(int,S)
    angle=getAngle(paperbot_message[2],paperbot_message[3])*(math.pi/180)
    z=[(paperbot_message[0]-20)/10,(paperbot_message[1]-20)/10,angle]
    
    K_diag=np.divide(np.array(P),np.array(R))
    K=np.array([[1-K_diag[0],0,0],[0,1-K_diag[1],0],[0,0,1-K_diag[2]]])
    
    
    state_est_a=apriori_state_estimate(state_est,paperbot_message[4])
    P_a=P #apriori covariance matrix is the same as the previous covariance matrix
    z_a=apriori_sensor_estimate(state_est_a)
    
    P= np.dot(K,P_a)
    
    state_est=state_est_a+np.dot(K,np.subtract(z,z_a))
    print(state_est)

def getAngle(mx,my): #use raw magnetometer readings to calculate angle. Set box up such that 62 degrees East is in the y direction.
    global angle
    if (my>=0 and my<=30) and (mx<200 and mx>170):
        angle=mapping(my,0,30,45,90)#angle from 45 to 90 
    elif (my>30 and my<=52) and (mx<190 and mx>=150):
        angle=mapping(my,30,52,90,180)#angle from 90 to 180
    elif (my<=52 and my>=0) and (mx<150 and mx>110):
        angle=mapping(my,52,0,180,270)#angle from 180 to 270
    elif (my>=-25 and my<0) and (mx<=150 and mx>110):
        angle=mapping(my,0,-25,270,359.9)#angle from 270 to 0
    elif (my>=-35 and my<0) and (mx>150 and mx<200):
        angle=mapping(my,-25,0,0,45)#angle from 0 to 45
    else:
        angle=angle
    return angle
        
def mapping(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def apriori_state_estimate(state_prev, Input):
    dt = 0.08 #time betweeen updates   
    v = 14.5    #velocity of wheels in cm/s
    w = 2*math.pi/2.2    #angular velocty in rad/s
    state_next_apriori = []
    x_prev = state_prev[0]
    y_prev = state_prev[1]
    theta_prev = state_prev[2]

        
    #forward
    if Input == 1:
        state_next_apriori.append( x_prev + dt*v*math.cos(theta_prev) )
        state_next_apriori.append( y_prev + dt*v*math.sin(theta_prev) )
        state_next_apriori.append( theta_prev )
    #reverse
    if Input == 2:
        state_next_apriori.append( x_prev - dt*v*math.cos(theta_prev) )
        state_next_apriori.append( y_prev - dt*v*math.sin(theta_prev) )
        state_next_apriori.append( theta_prev )
    #left
    if Input == 3:
        state_next_apriori.append( x_prev )
        state_next_apriori.append( y_prev )
        state_next_apriori.append( theta_prev - dt*w )
    #right
    if Input == 4:
        state_next_apriori.append( x_prev )
        state_next_apriori.append( y_prev )
        state_next_apriori.append( theta_prev + dt*w )
     #stationary
    if Input == 5:
        state_next_apriori.append( x_prev )
        state_next_apriori.append( y_prev )
        state_next_apriori.append( theta_prev )
    return state_next_apriori

#script for testing a priori sensor estimation

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
        return front_sensor_distance , right_sensor_distance, theta_apriori
    
    if theta_apriori == math.pi/2:
        front_sensor_distance = BOX_WIDTH - y_apriori
        right_sensor_distance = y_apriori        
        return front_sensor_distance , right_sensor_distance, theta_apriori
    
    if theta_apriori == math.pi:
        front_sensor_distance = x_apriori
        right_sensor_distance = y_apriori        
        return front_sensor_distance , right_sensor_distance, theta_apriori
    
    if theta_apriori == 3/2*math.pi:
        front_sensor_distance = y_apriori
        right_sensor_distance = y_apriori        
        return front_sensor_distance , right_sensor_distance, theta_apriori
    
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
            
   # print "positive dot products for front sensor: ",positive_dot_products_front
   # print " "
    
    #left wall dot product check
    if dot_products_front[0] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[0] is the smallest dot product
        for i in range(len(positive_dot_products_front)):
            if dot_products_front[0] > positive_dot_products_front[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            #print "enter left"
            front_sensor_distance = leftwall_distance_front;

    #right wall dot product check
    if dot_products_front[1] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[1] is the smallest dot product
        for i in range(len(positive_dot_products_front)):
            if dot_products_front[1] > positive_dot_products_front[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            #print "enter right"
            front_sensor_distance = rightwall_distance_front;
        
    #bottom wall dot product check
    if dot_products_front[2] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[2] is the smallest dot product        
        for i in range(len(positive_dot_products_front)):
            if dot_products_front[2] > positive_dot_products_front[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            #print "enter bottom"
            front_sensor_distance = bottomwall_distance_front;
         
    #top wall dot product check
    if dot_products_front[3] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[3] is the smallest dot product
        for i in range(len(positive_dot_products_front)):
            if dot_products_front[3] > positive_dot_products_front[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            #print "enter top"
            front_sensor_distance = topwall_distance_front;
        
    #determine which wall right sensor is pointing at    
    positive_dot_products_right = []
    for i in range(len(dot_products_right)):
        if dot_products_right[i] > 0:                              #store the positive dot products
            positive_dot_products_right.append(dot_products_right[i])
            
    #print "positive dot products for right sensor: ",positive_dot_products_front
    #print " "
    
    #left wall dot product check
    if dot_products_right[0] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[0] is the smallest dot product
        for i in range(len(positive_dot_products_right)):
            if dot_products_right[0] > positive_dot_products_right[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            #print "enter left"
            right_sensor_distance = leftwall_distance_right;

    #right wall dot product check
    if dot_products_right[1] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[1] is the smallest dot product
        for i in range(len(positive_dot_products_right)):
            if dot_products_right[1] > positive_dot_products_right[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            #print "enter right"
            right_sensor_distance = rightwall_distance_right;
        
    #bottom wall dot product check
    if dot_products_right[2] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[2] is the smallest dot product        
        for i in range(len(positive_dot_products_right)):
            if dot_products_right[2] > positive_dot_products_right[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            #print "enter bottom"
            right_sensor_distance = bottomwall_distance_right;
         
    #top wall dot product check
    if dot_products_right[3] > 0 :                              #first check if dot product is positive
        smallest_dot = 1                              #smallest_dot = 1 if dot_products[3] is the smallest dot product
        for i in range(len(positive_dot_products_right)):
            if dot_products_right[3] > positive_dot_products_right[i]:
                smallest_dot = 0       
        if smallest_dot == 1:
            #print "enter top"
            right_sensor_distance = topwall_distance_right;
 
    #print wall intersection distances 
    '''print " "
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
    '''
    return front_sensor_distance, right_sensor_distance, theta_apriori

    
    
    
    
def get_y_value(x_apriori, y_apriori, x ,slope):
    return slope*(x - x_apriori) + y_apriori

def get_x_value(x_apriori, y_apriori, y, slope):
    return ((y - y_apriori) / slope) + x_apriori

def dist(p1,p2):
    return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


def on_error(ws, error):
    print(error)

def on_close(ws):
    connected=False
    thread.exit()
    print("### closed ###")
    
def on_open(ws):
    print("connection opened")


if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://192.168.4.1:81/",
                              on_message = on_message,
                              on_error = on_error,
                              on_close = on_close)
    ws.on_open = on_open
    ws.run_forever()