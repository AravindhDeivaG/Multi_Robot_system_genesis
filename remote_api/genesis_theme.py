import sim # access all the sim elements
import sys 
import numpy as np 
import cv2
from PIL import Image
import array
import math
import time
from scipy.optimize import linear_sum_assignment as lsa 

clientID=-1 
wheel_handle={} #It is a 2D array of all 8 pairs of wheel handles of 8 bots

"""
For operating modes and return value constants: https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm
"""

def get_vision_sensor_image(clientID):

    """
    Purpose:
    ---
    This function should first get the handle of the Vision Sensor object from the scene.
    After that it should get the Vision Sensor's image array from the CoppeliaSim scene.

    Input Arguments:
    ---
    'clientID' : [integer]
        the client ID
    
    Returns:
    ---
    `vision_sensor_image` 	:  [ list ]
        the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
        the image resolution returned from the get vision sensor image remote API
    `return_code` 			:  [ integer ]
        the return code generated from the remote API
    
    Example call:
    ---
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()"""

    global vision_sensor_handle
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(clientID, vision_sensor_handle, 0, sim.simx_opmode_buffer)
    image_resolution.append(3)
    # print(image_resolution)
    # print(vision_sensor_image)

    vision_sensor_image = np.reshape(np.array(vision_sensor_image,np.uint8),image_resolution)
    vision_sensor_image = cv2.cvtColor(vision_sensor_image, cv2.COLOR_BGR2RGB)
    
    # np.array(vision_sensor_image,np.uint8) creates a numpy array of data type uint8
    # np.reshape(np.array(vision_sensor_image,np.uint8)) reshapes it into a 2D array of size (x,y) in the image resolution (512x512)
    # np.flip() Reverses the order of elements in an array along the given axis.The shape of the array is preserved, but the elements are reordered.
    # return np.flip(np.reshape(np.array(vision_sensor_image,np.uint8),image_resolution), 0)

    return np.flip(vision_sensor_image, 0)


def posFeedback(img):

    """
    Purpose:
    ---
    This function is called to get the position of all the bots currently in the arena

    Input Arguments:
    ---
    'img' : The image as a numpy array

    Returns:
    ---
    'resCentroids' : A list containing bots' [color+shape,[x,y]] as its entries
    """

    img_copy= img.copy()
    overall_lb= np.array([0,61,26])
    overall_ub= np.array([206,217,255])
    lb= np.array([80,143,117])
    ub= np.array([152,217,255])
    lg= np.array([45,87,82])
    ug= np.array([112,217,255])

    Centroids = []
    col='r'
    shape = ''
    x = -100
    y = -100

    hsv_img= cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask= cv2.inRange(hsv_img,overall_lb, overall_ub)
    _,threshold= cv2.threshold(mask, 127,255, cv2.THRESH_BINARY)
    mask_blue= cv2.inRange(hsv_img, lb,ub)
    mask_green= cv2.inRange(hsv_img, lg,ug)
    # cv2.imshow('Filtered_mask',threshold)
    contours,hierarchy1= cv2.findContours(threshold,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours :
        if cv2.contourArea(cnt)>100 :
            approx = cv2.approxPolyDP(cnt, 0.03* cv2.arcLength(cnt, True), True)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img_copy, [box],-1,(255,127,255),2)

            (x,y),r=cv2.minEnclosingCircle(cnt)
            # print(mask[int(y),int(x)])
            if mask_blue[int(y),int(x)]== 255:
                    col= 'b'
            elif mask_green[int(y),int(x)]== 255:
                    col= 'g'
            else:
                col = 'r'
            
            M1 = cv2.moments(cnt)
            M2 = cv2.moments(box)
            A1 = M1['m00']
            A2 = M2['m00']
            C = A1/A2
            print([x,y,A1,A2,C])
            if C >=0.83:
                print('{} -> Square at {},{}'.format(col,x,y))
                shape = 's'
                Centroids.append([col+shape,[x,y]])
            elif C <=0.6:
                print('{} -> Triangle at {},{}'.format(col,x,y))
                shape = 't'
                Centroids.append([col+shape,[x,y]])
            else :
                print('{} -> Circle at {},{}'.format(col,x,y))
                shape = 'c'
                Centroids.append([col+shape,[x,y]])
    print(Centroids)
    return(Centroids)



Iterm = {}
setpoint = {}
def init_handle(clientID):
    '''
    Return handles of revolute joints in a single dict (wheel_handle_dict)
    '''
    global Iterm
    img = get_vision_sensor_image(clientID)
    Centroids = posFeedback(img)

    wheel_handle_dict = {}
    base_handle_dict  = {}
    for c in Centroids:
        color_shape = c[0]
        lw_rev_joint_str = "lw_revolute_joint_" + color_shape
        rw_rev_joint_str = "rw_revolute_joint_" + color_shape
        base_handle_str  = "base_" + color_shape
        return_code , lw_joint_handle = sim.simxGetObjectHandle(clientID,lw_rev_joint_str,sim.simx_opmode_blocking)
        return_code , rw_joint_handle = sim.simxGetObjectHandle(clientID,rw_rev_joint_str,sim.simx_opmode_blocking)
        return_code , base_handle     = sim.simxGetObjectHandle(clientID,base_handle_str,sim.simx_opmode_blocking)
        sim.simxGetPingTime(clientID)
        wheel_handle_dict.update({color_shape:[lw_joint_handle,rw_joint_handle]})
        base_handle_dict.update({color_shape:base_handle})
        Iterm.update({color_shape:0})
        setpoint.update({color_shape:[]})
        sim.simxGetObjectOrientation(clientID,base_handle,-1,sim.simx_opmode_streaming)
        print(f"return_code : {return_code}")
    print(f"wheel_handle : {wheel_handle_dict}")
    print(f"base_handle : {base_handle_dict}")
    return [wheel_handle_dict,base_handle_dict]



"""
    simXFinish(clientID) : Ends the communication thread
    simXFinish(-1) : Ends all running communication thread
"""
sim.simxFinish(-1) # just in case, close all opened connections

"""
    number clientID = simxStart(string connectionAddress,number connectionPort,boolean waitUntilConnected,boolean doNotReconnectOnceDisconnected,number timeOutInMs,number commThreadCycleInMs)
        Starts a communication thread with the server (i.e. CoppeliaSim).

    Input Arguments:
    ---
    'connectionAddress' : the ip address where the server is located (i.e. CoppeliaSim)
    'connectionPort' : the port number where to connect. Specify a negative port number in order to use shared memory, instead of socket communication.
    'waitUntilConnected' : if True, then the function blocks until connected (or timed out).
    'doNotReconnectOnceDisconnected' : if True, then the communication thread will not attempt a second connection if a connection was lost.
    'timeOutInMs' :
        if positive: the connection time-out in milliseconds for the first connection attempt. In that case, the time-out for blocking function calls is 5000 milliseconds.
        if negative: its positive value is the time-out for blocking function calls. In that case, the connection time-out for the first connection attempt is 5000 milliseconds.
    'commThreadCycleInMs' : indicates how often data packets are sent back and forth. Reducing this number improves responsiveness, and a default value of 5 is recommended.

    Returns:
    ---
    'clientID' : the client ID, or -1 if the connection to the server was not possible (i.e. a timeout was reached).
"""
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # start a connection
print(clientID)


################################################################################################################

# Read input image to find the final vertices

# File name where the image is present
file_name_hexagon = "/home/aravindh/Documents/rmi/genesis/python_remote_api_code/hexagon.jpg"
# Determine required number of bots to occupy vertices of the given shape
img = cv2.imread(file_name_hexagon,0)
img = cv2.resize(img,(512,512))
cv2.imshow('',img)
cv2.waitKey(0)
_,threshold= cv2.threshold(img, 127,255, cv2.THRESH_BINARY)
contours,hierarchy1= cv2.findContours(threshold,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cnt = contours[0]
approx = np.squeeze(cv2.approxPolyDP(cnt, 0.03* cv2.arcLength(cnt, True), True))
n = len(approx) # Required number of bots

################################################################################################################

"""
    number returnCode=simxStartSimulation(number clientID,number operationMode)
        Requests a start of a simulation (or a resume of a paused simulation).

    Input Arguments:
    ---
    'clientID' : the client ID. refer to simxStart.
    'operationMode' : a remote API function operation mode. Recommended operation mode for this function is simx_opmode_oneshot.

    Returns:
    ---
    'returnCode' : a remote API function return code

"""
return_code = sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)



if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")


"""
    number returnCode,number handle=simxGetObjectHandle(number clientID,string objectName,number operationMode)
        Retrieves an object handle based on its name in CoppeliaSim

    Input Arguments:
    ---
    'clientID' : the client ID. refer to simxStart.
    'objectName' : name of the object.
    'operationMode' : a remote API function operation mode. Recommended operation mode for this function is simx_opmode_blocking

    Returns:
    --- 	
    'returnCode' : a remote API function return code
    'handle' : the handle
"""

return_code, vision_sensor_handle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking)
sim.simxGetPingTime(clientID)
"""
    number returnCode,array resolution,array image=simxGetVisionSensorImage(number clientID,number sensorHandle,number options,number operationMode)
        Retrieves the image of a vision sensor.

    Input Arguments:
    ---
    'clientID' : the client ID. refer to simxStart.
    'sensorHandle' : handle of the vision sensor
    'options' : image options, bit-coded:
    'bit0 set' : each image pixel is a byte (greyscale image), otherwise each image pixel is a rgb byte-triplet
    'operationMode' : a remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls)

    Returns:
    ---
    'returnCode' : a remote API function return code
    'resolution' : the resolution of the image (x,y)
    'image' : the image data.
"""
return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(clientID, vision_sensor_handle, 0, sim.simx_opmode_streaming)
sim.simxGetPingTime(clientID)

"""
    number returnCode,number pingTime=simxGetPingTime(number clientID)
        Retrieves the time needed for a command to be sent to the server, executed, and sent back.
        That time depends on various factors like the client settings, the network load, whether a simulation is running, whether the simulation is real-time, the simulation time step, etc.
        The function is blocking. This is a remote API helper function.
    
    Input Arguments:
    ---
    'clientID' : the client ID.

    Returns:
    ---	
    'returnCode' : a remote API function return code
    'pingTime' : a pointer to a simxInt value accepting the ping time in milliseconds.
"""
wheel_handle_dict , base_handle_dict = init_handle(clientID)


def differential_drive(v,w,color_shape):

    """
    This function is called to drive the wheels of the bot given by bot_id at the given linear and angular speed.
    The given linear and angular velocity is converted to accesible left wheel angular velocity and right wheel angular velocity using FBD of the bot.

    Input Arguments:
    ---
    'v' : Linear velociy needed to be imparted to the bot given by bot_id.
    'w' : angular velociy needed to 	'bot_id' :be imparted to the bot given by bot_id
    Number id within [0,7] to uniquely indentify the bot."""

    """
    number returnCode=simxSetJointTargetVelocity(number clientID,number jointHandle,number targetVelocity,number operationMode)
        Sets the intrinsic target velocity of a non-spherical joint.
        This command makes only sense when the joint mode is in torque/force mode: the dynamics functionality and the joint motor have to be enabled (position control should however be disabled)

    Input Arguments:
    ---
    'clientID' : the client ID. refer to simxStart.
    'jointHandle' : handle of the joint
    'targetVelocity' : target velocity of the joint (linear or angular velocity depending on the joint-type)
    'operationMode' : a remote API function operation mode. Recommended operation modes for this function are simx_opmode_oneshot or simx_opmode_streaming

    Returns:
    ---
    'returnCode' : a remote API function return code
    """

    global clientID, wheel_handle_dict
    R=0.05
    L=0.15
    l_motor_handle= wheel_handle_dict[color_shape][0]
    r_motor_handle= wheel_handle_dict[color_shape][1]
     
    w_l = ( (2.0 * v) - (w*L) ) / (2.0 * R)
    w_r = ( (2.0 * v) + (w*L) ) / (2.0 * R)

    w_upper = +np.pi*2/3
    w_lower = -np.pi*2/3
    
    if(w_l > w_upper):
        w_l = w_upper
    elif(w_l < w_lower):
        w_l = w_lower
    if(w_r > w_upper):
        w_r = w_upper
    elif(w_r < w_lower):
        w_r = w_lower
    print(f"w_l : {w_l} -- w_r : {w_r}")
    err_code = sim.simxSetJointTargetVelocity(clientID, l_motor_handle,w_l , sim.simx_opmode_oneshot)
    print(err_code)
    err_code = sim.simxSetJointTargetVelocity(clientID, r_motor_handle,w_r, sim.simx_opmode_oneshot)
    print(err_code)


################################################################################################################

#differential_drive(1,1,1)
LastTime=0
l_error=0
def controls(current_x,current_y,angle,color_shape):
    Kpv=0.02
    Kiv=0.02
    kpw=20
    global LastTime, l_error, v, w, Iterm, setpoint
    alpha, beta, g= angle
    
    g = g + np.pi/2
    if(g > np.pi):
        g = g - 2*np.pi
    
    setpoint_x,setpoint_y = setpoint[color_shape]
    slope=(setpoint_y-current_y)/(setpoint_x-current_x+0.000001)
    theta= math.atan(abs(slope))
    if setpoint_x>= current_x and setpoint_y>= current_y:
        theta=-theta
    elif setpoint_x< current_x and setpoint_y> current_y:
        theta= theta - np.pi
    elif setpoint_x< current_x and setpoint_y< current_y:
        theta= np.pi - theta 
    elif setpoint_x> current_x and setpoint_y< current_y:
        theta= theta
    else:
        pass
    
    a_error = theta - g

    if(a_error > np.pi):
        a_error = a_error - 2*np.pi
    elif(a_error < -np.pi):
        a_error = a_error + 2*np.pi

    d_error= math.sqrt(((setpoint_x-current_x)**2)+((setpoint_y-current_y)**2))
    d = d_error
    w= kpw* a_error

    print(f"current_orientation : {g*180/np.pi}")
    print(f"set_orientation     : {theta*180/np.pi}")
    print(f"angle_error          : {a_error*180/np.pi}")
    print(f"distance_error          : {d_error}")
    
    # added weight on angular velocity
    if(np.abs(a_error*180/np.pi) >= 25):
        d_error = 0
    else:
        pass

    SetTime = 0.05 # 1sec
    now = time.perf_counter()
    time_change = now - LastTime
    if time_change >= SetTime:
        Iterm[color_shape] += (Kiv * d_error * SetTime) / 100

        v = abs(Kpv * d_error + Iterm[color_shape] )
        
        l_error = d_error
        LastTime = now
    print("v, w:",v,w)
    return v, w, d
    

################################################################################################################
# Assignment problem : Assigning am emd coordinate to every bot
img = get_vision_sensor_image(clientID)
Centroids = posFeedback(img)
p1 = []
p2 = approx
M = np.zeros((n,n))
for i in Centroids:
    p1.append(i[1])
p1 = np.array(p1)
print(p1)
print(p2)
for i in range(n):
    for j in range(n):
        d = np.sqrt(np.sum(np.square(p1[i]-p2[j])))
        M[i][j]+=d
print(M)
col_ind,row_ind = lsa(M)
print(f"col_ind : {col_ind}")

for i in range(n):
    c = Centroids[i]
    color_shape = c[0]
    index = row_ind[i]
    x,y = p2[index]
    setpoint[color_shape] = [x,y]

################################################################################################################


while True:
    img = get_vision_sensor_image(clientID)
    Centroids = posFeedback(img)
    img_copy = img.copy()
    for i in setpoint.values():
        x,y = i
        cv2.circle(img_copy,(x,y),30,(255,255,255),3)
    cv2.imshow("",img_copy)
    cv2.waitKey(1)
    i = 0 # Variable to count the number of bots that reached it's setpoint
    i_max = len(Centroids) # Total number of bots
    for c in Centroids:
        color_shape = c[0]   
        x,y= c[1]
        code,angle = sim.simxGetObjectOrientation(clientID,base_handle_dict[color_shape],-1,sim.simx_opmode_buffer)
        v,w,d_error= controls(x,y,angle,color_shape)
        if(d_error<=15):
            print("******************************")
            print(f"bot {color_shape} reached it's setpoint")
            print("******************************")
            i+=1
            differential_drive(0,0,color_shape)
        else:
            differential_drive(v,w,color_shape)
    if(i==i_max):
        break
    time.sleep(0.1)

cv2.waitKey(2000)
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
sim.simxGetPingTime(clientID)
sim.simxFinish(-1)

# while (sim.simxGetConnectionId(clientID)!=-1):
# 	img = get_vision_sensor_image(clientID)
# 	cv2.imshow('', img)
cv2.waitKey(1)
cv2.destroyAllWindows()



