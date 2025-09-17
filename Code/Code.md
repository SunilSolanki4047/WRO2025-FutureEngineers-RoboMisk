 Below is the Obstacle Challenge Code and its explanation: 
---
from picamera2 import Picamera2

import cv2

import numpy as np

import time

import pigpio

import datetime

import board

import adafruit_bno055

import RPi.GPIO as GPIO
---
[The above consists of all the libraries and modules utilised by the team in the code. 

Picamera2: Allows us to communicate with the Picam 3 and set values such as exposure and frame rate

cv2: A vision software allowing us to perform image manipulation, draw contours and calculate the area, x, y and other factors relating to the object

numpy: Allows the team to store HSV values in arrays

time: Allows the team to add time based movements and set timers

pigpio: Allows the team to control the motors and servos accurately

datetime: Allows the team to capture the camera preview, and save it with the current date and time

board: Allows 12C communication

adafruit_bno055: Allows the team to communicate and gain data drom the IMU

RPi.GPIO: Allows the team to directly attach IR sensors and Limit Switches to the GPIO pins easily]

# === I2C & sensors ==
i2c = board.I2C()

sensor = adafruit_bno055.BNO055_I2C(i2c)

[IMU setup]

offset = 0

prev_heading = None

total_heading = 0

(Global variables for IMU)

# === Pin and hardware config ===
MOTOR_PIN_1 = 24 #Motor Driver PIN1

MOTOR_PIN_2 = 23 #Motor Driver PIN2

MOTOR_ENA = 18 #Motor Driver Enable Pin

SERVO_PIN = 25 #Servo Out Pin

BUTTON_START = 14 #Button 1 Digital Input Pin

BUTTON_STOP = 15 #Button 2 Digital Input Pin

[Teams may change these pins as per their needs]

# Motor PWM frequency
MOTOR_PWM_FREQ = 1000

BUTTON_PIN = 26  # Limit Switch (Left) Out Pin

GPIO.setmode(GPIO.BCM)

GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) # (Limit Switch (Left) Setup

GPIO.setup(12,GPIO.IN,pull_up_down=GPIO.PUD_UP) # (Limit Switch (Right) Setup)

GPIO.setup(21,GPIO.IN) # (IR Sensor Right Back Setup)

GPIO.setup(20,GPIO.IN) # (IR Sensor Right Front Setup)

GPIO.setup(16,GPIO.IN) # (IR Sensor Left Front Setup)

GPIO.setup(19,GPIO.IN) # (IR Sensor Left Back Setup)


[Teams may change the pin numbers as per their requirements] 

# Speed settings (0-255 PWM duty cycle)

SPEED_RUN = 80 

SPEED_RUN_SLOW = 75

SPEED_STOP = 0

[Teams may change the SPEED_RUN to make their robot faster or slower]
# Steering angles (degrees)

STEER_CENTER = 110

STEER_LEFT_LIMIT = 78

STEER_RIGHT_LIMIT = 142

[The above are variables referring to the positions the servo is at, at the centre, left and right]

# Steering adjustment factors (tuning)

STEER_FACTOR_GREEN_HIGH =0.09

STEER_FACTOR_GREEN_LOW = 0.07

STEER_FACTOR_RED_HIGH = 0.09

STEER_FACTOR_RED_LOW = 0.07

STEER_FACTOR_BLACK_DIFF = 0.07

STEER_FACTOR_SINGLE_TARGET = 0.08

STEER_FACTOR_OUTER_ROUND=0.04

STEER_FACTOR_LEFT_Y_DIFF = 0.07

[The above are Kp values that teams shall have to adjust based on their robot for different aspects of the code]
# Camera settings

EXPOSURE_TIME =13000 #Brightness setting for the PiCam3

X_RESOL = 1080 #Screen width

Y_RESOL = 300 #Screen Length

X_MID = X_RESOL / 2 #Middle of the screen

LEFT_LIMIT = 10 #Lefmost limit of the screen

RIGHT_LIMIT = X_RESOL - 10 #Rightmost limit of the screen

[Teams may change the X_RESOL or Y_RESOL, if not enough is visible from a point or if too many objects are being detected]

# HSV thresholds for colors
GREEN_LOWER = np.array([50, 100, 50])

GREEN_UPPER = np.array([70, 255, 255])

RED_LOWER1 = np.array([0, 120, 50])

RED_UPPER1 = np.array([10, 255, 255])

RED_LOWER2 = np.array([160, 120, 50])

RED_UPPER2 = np.array([165, 255, 255])

PINK_LOWER = np.array([160, 100, 120])

PINK_UPPER = np.array([175, 255, 255])

BLACK_LOWER = np.array([0, 0, 0])

BLACK_UPPER = np.array([179, 255, 50])

BLUE_LOWER = np.array([100, 150, 50])
BLUE_UPPER = np.array([130, 255, 255])

ORANGE_LOWER = np.array([10, 150, 100])

ORANGE_UPPER = np.array([25, 255, 255])

PURPLE_LOWER = np.array([160, 80, 40])

PURPLE_UPPER = np.array([175, 255, 200])

[The above are the Hue, Saturation and Value features required for the detection of the black walls, obstacle, orange and blue lines and parking zone. If the HSV values are not sufficient for the given detection, teams may modify them]

# Line detection parameters
LINE_COOLDOWN = 3.5  # seconds cooldown before recounting same line [Teams may adjust this in case the lines are being detected repeatedly or not at all]

LINE_DETECT_REGION_Y =Y_RESOL-50  # y coordinate near bottom to detect line crossing [Teams may adjust this distance based on their camera position]
 
ParkOver=False # [Our team starts in the parking zone, thus, this variable signals for the exit manuever to start, teams may change this to true if they do not wish to start in the parking zone]


# Line counting mode: 'blue', 'orange', or 'both'
LINE_COUNT_MODE = 'both'  # Change this to 'blue' or 'orange' as needed [For redundancy, the team takes the highest count of the line detected, (eg- Orange count=10, Blue Line count=9, the round count shall be taken as 10. If the team wishes to only look for 1 line (orange or blue), they may change it to 'orange' or 'blue' accordingly]

# === Initialize PiCamera2 ===
picam2 = Picamera2()

preview_config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (X_RESOL, Y_RESOL)})

picam2.preview_configuration.controls.FrameRate = 60 #Teams may increase this to a maximum of 660 fps if they feel detection rate is not fast enough.]

picam2.configure(preview_config)

picam2.set_controls({

    "AeEnable": False,
    
    "ExposureTime": EXPOSURE_TIME,
    
    "AnalogueGain": 4.0
})

timestamp = datetime.datetime.now() # [The team would save the preview of the video recorded by the camera, and would be saved by the time of recording. In case teams feel they do not need this, they may decided to remove it]

video = cv2.VideoWriter(f"output{timestamp}.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 20, (X_RESOL, Y_RESOL))

picam2.start() #Start camera

time.sleep(1) # Note- [The camera takes a minimum of 1 second to setup, do not decrease the value]

# === Setup GPIO pins ===
pi = pigpio.pi() 

pi.set_mode(BUTTON_START, pigpio.INPUT) # [Setup for the button pin1]

pi.set_pull_up_down(BUTTON_START, pigpio.PUD_UP) # [Setmode for button pin1]

pi.set_mode(BUTTON_STOP, pigpio.INPUT) # [Setup for the button pin2]

pi.set_pull_up_down(BUTTON_STOP, pigpio.PUD_UP) # [Setmode for button pin2]

for pin in (MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_ENA): # [Setup of motor pins]
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)

pi.set_PWM_frequency(MOTOR_ENA, MOTOR_PWM_FREQ) # [PWM frequency for speed setting]

# === Robot control functions ===
def steer(angle: float): #[Set angle for servo position function]

    """Set servo angle within limits."""
    
    angle = max(STEER_LEFT_LIMIT, min(STEER_RIGHT_LIMIT, angle)) 
    
    #[In any case if the angle written in the function is less or greater than the limit, the angle is set accordingly either to the STEER_LEFT_LIMIT or STEER_RIGHT_LIMTI ]
    
    pulse = int(500 + (angle / 180) * 2000) [Conversion from pulse to degrees, requesting teams not to change this]
    
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)
    
    # [The above function allows one to easily set the angle of the servo by calling the steer() function]

def run(speed: int): # [Run motor forward at a given speed]

    """Run motors forward at speed (0-255)."""
    
    pi.write(MOTOR_PIN_1, 1) #Teams may have to reverse this based on their MOTOR_PIN_1 and MOTOR_PIN_2 connections
    
    pi.write(MOTOR_PIN_2, 0)
    
    pi.set_PWM_dutycycle(MOTOR_ENA, speed)
    
#The above function allows one in the code to set the speed for the forward direction by calling the function run()

def back(speed: int): [#Run motor backward at a given speed]

    """Run motors backward at speed (0-255)."""
    
    pi.write(MOTOR_PIN_1, 0)  #Teams may have to reverse this based on their MOTOR_PIN_1 and MOTOR_PIN_2 connections
    
    pi.write(MOTOR_PIN_2, 1)
    
    pi.set_PWM_dutycycle(MOTOR_ENA, speed)

#The above function allows one in the code to set the speed for the reverse direction by calling the function back()

def stop_motors():

    """Stop all motors."""

    pi.set_PWM_dutycycle(MOTOR_ENA, 0)
    
    pi.write(MOTOR_PIN_1, 0)
    
    pi.write(MOTOR_PIN_2, 0)

    # The above function stops the motors and locks them in place and can be called by the function stop_motors()
    
def reset_heading():

    global prev_heading,total_heading
    
    prev_heading = None

    total_heading = 0

    #In order to reset the IMU before parking, this function may be called. The total heading is reset back to 0 allowing for accurate data readings. This function may be called by reset_heading()
    
def get_heading():

    global total_heading,prev_heading
    
    try:
        time.sleep(0.03) # [Requesting teams to not edit this, as this delay may cause great changes in sensor readings]
        
        current_heading = sensor.euler[0] # [The heading is returns other values such as acceleration and speed as well, [0], gives access to the angle]
        
        if current_heading is None:
            return total_heading # [If no reading is available, return 0]

        if prev_heading is None: # [Keep updating prev_heading, in order to get the change in heading (delta)]
            prev_heading = current_heading
            return total_heading

        delta = current_heading - prev_heading
        if delta > 180: #[Limit delta to 180 and -180)
            delta -= 360
        elif delta < -180:
            delta += 360

        total_heading += delta #[Keep updating total heading]
        prev_heading = current_heading
        return total_heading #[return the value]
    except Exception as e: # [In case there is an error in the reading, simply return the total_heading]
        print("Error reading heading:", e) 
        return total_heading
    # [The above function allows one to gain the angle of the robot in terms of degrees accurately by calling the function get_heading()]
def get_red_mask(hsv):

    """Get combined red mask handling hue wrap."""
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    return cv2.bitwise_or(mask1, mask2)
    # [The above function allows for a more neat and systematic detection by using functions. Teams should be advised to not change this. Function is called by get_red_mask()]
    
# === Global variables for line counts and cooldown ===
blue_line_count = 0 #How many times blue line is detected

orange_line_count = 0 #How many times orange line is detected

last_blue_time = 0 #Time when the blue line was last detected

last_orange_time = 0 #Time when the orange line was last detected

[In order to avoid detecting the same line repeatedly a time based logic is used which limits the camera reading to the line]

# Rounds & stopping control
rounds_completed = 0 #Currently how many rounds have been completed

insidePark=True #Is the robot currently between the parallel walls? Note- [If it is not wished to start in parking zone, the variable may be set to 'False']

targ1 = True #Completion of turn 1

targ2 = True #Completion of turn 2

targ3=True #Completion of turn 3

targ4=True # Completion of turn 4

     # [targ1,2,3,4 ensure that the turns to exit the parking zone only occur once after which, the next turn happens]

selected = False # Is direction set?
    # [The team establishes direction on the basis of the position of the black walls in the very start, after which it is set to 'True' to ensure this process occurs only once. If teams wish to preset a direction, they may set selected to 'True' and accordingly set CLOCKWISE as 'true' or 'false'

CLOCKWISE = True # default direction

over = False # Should the robot start parking?  # [If teams wish to only test the parking, they may set over to 'True']

parked = False # Is parking done? # [If teams wish to disable the parking, they may set parked to 'True']

# === Main processing function ===
def process_frame():

    global ButtonTime, Reset, STEER_FACTOR_OUTER_ROUND
    
    global blue_line_count, orange_line_count, last_blue_time, last_orange_time,purple_target
    
    global rounds_completed,selected,CLOCKWISE
    
    global targ1,targ2,insidePark,targ3,targ4,parked,over

    #Use the variables set in initialization
    
    frame = picam2.capture_array() #Set the frame for the picamera
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Set hsv value on the basis of the frame
    
    blur = cv2.GaussianBlur(hsv, (5, 5), 0) #Apply a Gaussian blur to eliminate random detections

    #Note- [Teams are advise to not change the frame, hsv and blur variables]
    
    current_time = time.time() #[Constantly set the timer functoion in the code in order for the line debouncing logic]
    
    black_mask = cv2.inRange(blur, BLACK_LOWER, BLACK_UPPER)
    
    green_mask = cv2.inRange(blur, GREEN_LOWER, GREEN_UPPER)
    
    red_mask = get_red_mask(hsv)
    
    pink_mask = cv2.inRange(blur, PINK_LOWER, PINK_UPPER)
    
    blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    
    orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    
    purple_mask = cv2.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER)
    
    kernel_5 = np.ones((5, 5), np.uint8)
    
    kernel_50 = np.ones((50, 50), np.uint8)

    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel_50)
    
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_DILATE, kernel_50)
    
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel_5)
    
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel_5)

    #The masks set the hsv range and values as defined earlier in the initialization process. The kernels allow for object definition and setting the centre point of the object detected.

    # Blue line detection
    global ParkOver
    blue_detected = False
    blueDone=0
    if LINE_COUNT_MODE in ('blue', 'both'):
        contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # draw a contour over the blue line detected
        for cnt in contours_blue:
            if cv2.contourArea(cnt) > 1000 and ParkOver: # [only detect if getting out of the parking zone is over and the min. area for detection > 1000]
                x, y, w, h = cv2.boundingRect(cnt)
                if y + h >= LINE_DETECT_REGION_Y: # [Only within a certain distance should the contour be made]
                    blue_detected = True
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    break

    # Orange line detection
    orange_detected = False
    orangeDone=0
    if LINE_COUNT_MODE in ('orange', 'both'):
        contours_orange, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # draw a contour over the orange line detected
        for cnt in contours_orange:
            if cv2.contourArea(cnt) > 1000 and ParkOver:  # [only detect if getting out of the parking zone is over and the min. area for detection > 1000]
                x, y, w, h = cv2.boundingRect(cnt)
                if y + h >= LINE_DETECT_REGION_Y: # [Only within a certain distance should the contour be made]
                    orange_detected = True
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 140, 255), 2)
                    break

    if blue_detected and (current_time - last_blue_time) > LINE_COOLDOWN:
    
    #only register if the line has been detected if the time between the previous detection and the current time is greater than the cooldown time]
    
        if GPIO.input(26)==GPIO.LOW or GPIO.input(12)==GPIO.LOW:
        
        """Since the limit switches are generally triggered at the corner, and thus due to the reverse, we count the lines twice, we added a line_count depletion condition to avoid miscount. If teams find this to not be a problem they may remove this]"""
        
            blue_line_count-=1
            
        blue_line_count += 1
        
        last_blue_time = current_time
        
        print(f"Blue line crossed! Count: {blue_line_count}")

    if orange_detected and (current_time - last_orange_time) > LINE_COOLDOWN:
    
       #only register if the line has been detected if the time between the previous detection and the current time is greater than the cooldown time]
       
        if GPIO.input(26)==GPIO.LOW or GPIO.input(12)==GPIO.LOW:
                """Since the limit switches are generally triggered at the corner, and thus due to the reverse, we count the lines twice, we added a line_count depletion condition to avoid miscount. If teams find this to not be a problem they may remove this]"""
                
            orange_line_count-=1
            
        orange_line_count += 1
        
        last_orange_time = current_time
        
        print(f"Orange line crossed! Count: {orange_line_count}")
    
    # Calculate rounds completed
    if LINE_COUNT_MODE == 'blue':
    
        rounds_completed = blue_line_count

    elif LINE_COUNT_MODE == 'orange':
    
        rounds_completed = orange_line_count
        
    else:  # both
    
        rounds_completed = max(blue_line_count, orange_line_count) # [Condition for maximum round count]

    # [The above condition allows the classification of the line count on the basis of 'both' or 'orange' or 'blue' 

    
    # Find black contours for navigation
    contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    left_target = None
    
    right_target = None
    
    max_left_y = 0 # Detection for left wall
    
    max_right_y = 0 #Detection for right wall

    for contour in contours:
    
        area = cv2.contourArea(contour)
        
        if area > 1000:
        
            x, y, w, h = cv2.boundingRect(contour)
            
            cx = int(x + w / 2)
            
            cy = int(y + h / 2)
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 105, 255), 2) # [Draw a contour around the middle of the detected portion]
            
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1) # [Draw a dot indicating the centre]
            
            if not selected:
            # [Classification of direction- clockwise or anticlockwise]

                """ [Since we start inside the parking zone, the right wall is always in closer to us, thus, on the basis of where the larger black contour is detected, we may establish the direction]"""
                
                if cx < X_MID: # [Our screen ranges from 0 to 1080, from the left to right. Thus, if the detected portion is on the left, direction is clockwise]
                
                    CLOCKWISE = True
                else: # [ if the detected portion is on the left, direction is clockwise, else it must be anticlockwise [CLOCKWISE='False' ]]
                
                    CLOCKWISE = False
                    
                selected = True # [The process of find direction should only occur once.]
                
            if CLOCKWISE: #If direction is CLOCKWISE
                if cx <= X_MID + 10 and cy > max_left_y:
                
                """[X_MID+10 gives some error range for camera fluctuations, teams may have to increase or decrease this. cy>max_left_y ensures only the closest reading is taken into consideration after which its value (initially 0), is updated to the current cy]"""
                
                    max_left_y = cy
                    
                    left_target = (x + w, y + h) # (cx and cy values. since the y+h of the wall gives overall cy and similarly for cx)
                    
                elif cx > X_MID + 10 and cy > max_right_y: 
                ("Opposite if detectrd refion is in the right)
                
                    max_right_y = cy
                    
                    right_target = (x, y + h) # (Since the target is on the right, no additional w is needed)
                    
            else: 
            
                    """[X_MID-10 gives some error range for camera fluctuations, teams may have to increase or decrease this. cy>max_left_y ensures only the closest reading is taken into consideration after which its value (initially 0), is updated to the current cy]"""
                
                if cx >= X_MID - 10 and cy > max_right_y: 
                
                    max_right_y = cy
                    
                    right_target = (x, y + h) # (Since the target is on the right, no additional w is needed)
                    
                elif cx < X_MID - 10 and cy > max_left_y: 
                
                    max_left_y = cy
                    
                    left_target = (x + w, y + h) # (Since the target is on the left, an additional w is required)
                
    # Detect green obstacles
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    green_target = None
    
    green_y = 0
    
    for c in green_contours:
    
        area = cv2.contourArea(c)
        
        if 1000 < area <= 20000: # Ensure that the detected area is not a wall or some other object
        
            x, y, w, h = cv2.boundingRect(c)
            
            cx = x # Since we have to go to the left of green, we simply take cx as x.
            
            cy = y + h #The object's h must also be taken into consideration for the total y distance
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) # Draw a contour around the detected region
            
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1) # Draw a dot at the bottom left corner
            
            cv2.putText(frame, f"Green Obstacle: {cy}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2) #print the cy on the screen
            
            if CLOCKWISE:
                if cy > green_y: # Detect only the closest obstacle
                    green_y = cy
                    green_target = (cx, cy)
                    if rounds_completed >= 12:
                        right_target = (cx, cy) # [We take an outer left round before park. Due to this, during that we establish any obstacle as a right wall]
            else:
                if cy > green_y:  # Detect only the closest obstacle
                    green_y = cy
                    green_target = (cx, cy)
                    if rounds_completed >= 12:
                        left_target = (cx, cy) # [We take an outer right round before park. Due to this, during that we establish any obstacle as a left wall]
            
    # Detect red obstacles
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_target = None
    red_y = 0
    for c in red_contours:
        area = cv2.contourArea(c)
        if area > 1000: #Ensure a minimum area of detection
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w #Since we have to go on the right side, we add an offset of w, the objects pixel width as seen on the screen
            cy = y + h
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2) #Draw a contour around it
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1) # Put a point on the bottom right corner
            cv2.putText(frame, f"Red Obstacle:{cy}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2) # print the cy on the screen
            if CLOCKWISE:
                if cy > red_y:
                    red_y = cy
                    red_target = (cx, cy)
                    if rounds_completed >= 12:
                        right_target = (cx,cy)  # [We take an outer left round before park. Due to this, during that we establish any obstacle as a right wall]
            else:
                if cy > red_y :
                    red_y = cy
                    red_target = (cx, cy)
                    if rounds_completed >= 12:
                        left_target = (cx,cy) # [We take an outer right round before park. Due to this, during that we establish any obstacle as a left wall]

    current_time = time.time()
    # start the timer once again
    purple_detected = False
    purple_target = None
    
    contours_purple, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_purple:
        x, y, w, h = cv2.boundingRect(cnt)         
        if cv2.contourArea(cnt) > 1000: # Ensure a minimum areA
            if CLOCKWISE:
                purple_detected = True
                cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 105, 255), 2) # draw contour around the detected region
                cv2.circle(frame, (x+w, y), 5, (255, 0, 0), -1) # draw a point on the top right corner
                left_target = (x+w, y) # In case the rounds are still going on, set the purple area as black wall on the left (CLOCKWISE)
                purple_target = (x+w, y) # Set purple's reference point as top right corner on the wall
            else:
                purple_detected = True
                cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 105, 255), 2) # draw contour around the detected region
                cv2.circle(frame, (x, y), 5, (255, 0, 0), -1) # draw a point on the top left corner of the wall
                right_target = (x, y) # In case the rounds are still going on, set the purple area as black wall on the right (ANTICLOCKWISE)
                purple_target = (x, y) # Set purple's reference point as top left corner on the wall
            
    if not over: # if the 4th round is not over and purple has not been detected
        print("Heading : ",get_heading()) # print heading for redundancy . Teams may remove this
        if insidePark: # if robot is inside the parking zone
            if CLOCKWISE:
                if abs(get_heading()) < 20 and targ1: # turn1 towards the left, slightly backwards (Decrease if hitting the back wall, increase if too less)
                    steer(72)
                    back(85)
                elif abs(get_heading()) < 50 and targ2: #turn2 towards the right, forwards (Increase if too less, decrease if too much)
                    targ1 = False
                    steer(148)
                    run(85)
                elif abs(get_heading())>5 and targ3: # turn3 towards the left forwards (Decrease if too much, Increase if too less)
                    targ2=False
                    steer(72)
                    run(85)
                else:
                #Note-[Teams shall have to change the 20,50 and 5 values based on the imu position, robot's length and imu orientation]
                    targ3=False
                    if targ3==False and targ4:
                        run(0)
                        steer(110)
                        time.sleep(0.5)
                        back(100) #come back for a fixed duration to avoid accidentally crossing over an obstacle
                        time.sleep(1.1) # (teams may have to change this value)
                        targ4=False
                        insidePark=False
                        time.sleep(0.5)
                        stop_robot()
                        
            else:
                if abs(get_heading()) <15 and targ1: # turn1 towards the right, slightly backwards (Decrease if hitting the back wall, increase if too less)
                    steer(148)
                    back(85)
                
                elif abs(get_heading()) <50 and targ2: #turn2 towards the left, forwards (Increase if too less, decrease if too much)
                    targ1 = False
                    steer(72)
                    run(85)
                elif abs(get_heading()) >10 and targ3: # turn3 towards the right forwards (Decrease if too much, Increase if too less)
                    targ2=False
                    steer(148)
                    run(85)                                 
                else:
                 #Note-[Teams shall have to change the 15,50 and 10 values based on the imu position, robot's length and imu orientation]
                    targ3=False
                    targ2 = False
                    if targ3==False and targ4:
                        run(0)
                        steer(110)
                        time.sleep(0.5)
                        back(100) #come back for a fixed duration to avoid accidentally crossing over an obstacle
                        time.sleep(1.1) # (teams may have to change this value)
                        targ4=False
                        insidePark=False
                        run(0)
                        time.sleep(0.45)
                        stop_robot()
                        time.sleep(0.5)
                        ParkOver=True
                        
        elif GPIO.input(12)==GPIO.LOW: # If the right limit switch is pressed, the robot shall turn towards the right and come back slightly for redundancy
            run(0)
            time.sleep(0.3)
            steer(STEER_RIGHT_LIMIT) 
            time.sleep(0.4)
            back(90)
            time.sleep(0.5) # teams may have to change the back time
            run(0)
            time.sleep(0.4)
            steer(110)
            time.sleep(0.4)

        elif GPIO.input(26)==GPIO.LOW:
            run(0)
            time.sleep(0.3)
            steer(STEER_LEFT_LIMIT)
            time.sleep(0.4)
            back(90) # teams may have to change the back time
            time.sleep(0.5)
            run(0)
            time.sleep(0.4)
            steer(110)
            time.sleep(0.4)
      #The above GPIO 26 and GPIO 12 functions are for backup purposes. Teams can remove them if they wish to.

        elif purple_detected and rounds_completed >= 14: # if the outer round is completed and purple is detected
            run(60)
            only_x, only_y = purple_target
            if CLOCKWISE: # Keep Purple On left when clockwise
                steer(STEER_CENTER + ((only_x - 420) * STEER_FACTOR_SINGLE_TARGET))# change the servo angle from centre to required angle)
            else: #keep purple on right when anticlockwise
                steer(STEER_CENTER + ((only_x - 750) * STEER_FACTOR_SINGLE_TARGET))# change the servo angle from centre to required angle)
            if (GPIO.input(21)==GPIO.LOW) or (GPIO.input(16)==GPIO.HIGH): 
            
            """Detection via IR Sensor. Since the IR sensor cannot detect the black walls, but can detect the pink walls, after the outer round is over and the camera has seen the parking zone, we may reliably move forward via the IR sensor"""
            
                over = True # Start parking
                steer(STEER_CENTER)
                stop_robot()
                time.sleep(2) 

                    
        elif green_target and red_target and rounds_completed <=12: # if rounds completed is less than 13 and both red and greeen is detected
            run(SPEED_RUN)
            if green_y > red_y: # (If green is closer than red, follow green)
                right_x, right_y = green_target
                if right_y > 100: # (Until the robot is not within a certain range, keep green in the centre of the screen and follow (540))
                    steer(STEER_CENTER + ((right_x - 820) * STEER_FACTOR_GREEN_HIGH)) 
                    # (In order to go to the left, green must be on the right, thus, the error is green's current x - the required position (820 on the screen)
                    # (This error is multiplied by a constant to give an output servo position)
                else:
                    steer(STEER_CENTER + ((right_x - 540) * STEER_FACTOR_GREEN_LOW))
                    #(Until the robot is close enough, try to set the obstacle in the centre of the screen)
            else:
                left_x, left_y = red_target
                if left_y > 100: #(Until the robot is not within a certain range, keep green in the centre of the screen and follow (540))
                    steer(STEER_CENTER + ((left_x - 260) * STEER_FACTOR_RED_HIGH))
                    # (In order to go to the right, red must be on the left, thus, the error is red's currrent x- the required position (260 on the screen)                            # (This error is multiplied by a constant to give an output servo position)    
                else:
                    steer(STEER_CENTER + ((left_x - 540) * STEER_FACTOR_RED_LOW))
                    #(Until the robot is close enough, try to set the obstacle in the centre of the screen)
                    
        elif green_target and rounds_completed <= 12:
            run(SPEED_RUN)
            if CLOCKWISE:
                right_x, right_y = green_target
                if right_y > 100:
                    steer(STEER_CENTER + ((right_x - 820) * STEER_FACTOR_GREEN_HIGH))
                else:
                    steer(STEER_CENTER + ((right_x - 540) * STEER_FACTOR_GREEN_LOW))
            else:
                right_x, right_y = green_target
                if right_y > 100:
                    steer(STEER_CENTER + ((right_x - 820) * STEER_FACTOR_GREEN_HIGH))
                    print("920")
                else:
                    print("540")
                    steer(STEER_CENTER + ((right_x - 540) * STEER_FACTOR_GREEN_LOW))
        """[The above snipet displays the logic used for obstacle avoidance. We must deviate to the left of the green obstacle. Thus, to our screen, the obstacle must appear on the right. Therefore, out error is the obstacle's current x- the required x (rightx-820) which is multiplied by a Kp (STEER_FACTOR_GREEN_HIGH) and added to the servo's position. Why is it added? Simple, since the servo's centre is 110, and deviations shall have to be added to it. eg( 110 +(-20)) would lead to the servo turning to the 90 position, making our robot move towards the left. Similiarly, if the y of the obstacle is less than 100 (far away), we try and shift the obstacle to the centre of the screen to ensure our robot remain straight. """

"""  [Notice this, both the clockwise and anticlockwise portions are exactly the same! Why? Since we have to go to the left of the green obstacle, and since the preview screen gets practically flipped upon directional switch, there is no need to make any logical changes in the code. However, we still keep the portion in order for any last minute shift changes (820) required."""

"""Teams are advised to change the 820 and 260 shift values as per there needs."""
        

        elif red_target and rounds_completed <= 12:
            run(SPEED_RUN)
            if CLOCKWISE:
                left_x, left_y = red_target
                if left_y > 100:
                    steer(STEER_CENTER + ((left_x - 260) * STEER_FACTOR_RED_HIGH))
                else:
                    steer(STEER_CENTER + ((left_x - 540) * STEER_FACTOR_RED_LOW))
            else:
                left_x, left_y = red_target
                if left_y > 100:
                    steer(STEER_CENTER + ((left_x - 260) * STEER_FACTOR_RED_HIGH))
                else:
                    steer(STEER_CENTER + ((left_x - 540) * STEER_FACTOR_RED_LOW))
            """[The above snipet displays the logic used for obstacle avoidance. We must deviate to the right of the red obstacle. Thus, to our screen, the obstacle must appear on the left. Therefore, out error is the obstacle's current x- the required x (rightx-260) which is multiplied by a Kp (STEER_FACTOR_RED_HIGH) and added to the servo's position. Why is it added? Simple, since the servo's centre is 110, and deviations shall have to be added to it. eg( 110 +(+20)) would lead to the servo turning to the 130 position, making our robot move towards the right. Similiarly, if the y of the obstacle is less than 100 (far away), we try and shift the obstacle to the centre of the screen to ensure our robot remain straight. """

"""  [Notice this, both the clockwise and anticlockwise portions are exactly the same! Why? Since we have to go to the right of the red obstacle, and since the preview screen gets practically flipped upon directional switch, there is no need to make any logical changes in the code. However, we still keep the portion in order for any last minute shift changes (260) required."""

"""Teams are advised to change the 820 and 260 shift values as per there needs."""          

        else:
            run(SPEED_RUN)
            if left_target and right_target:
                left_x, left_y = left_target
                right_x, right_y = right_target
                if left_y - right_y > 80 and not rounds_completed>12:# if left wall is clearly closer than the right wall
                    steer(STEER_CENTER + (left_x * STEER_FACTOR_LEFT_Y_DIFF))# Go towards the right wall. (Since Error is taken as the left walls' x)
                elif right_y - left_y > 80 and not rounds_completed>12:
                    steer(STEER_CENTER + (right_x * STEER_FACTOR_LEFT_Y_DIFF)) 
          #Move towards the (Error=right wall x) until the left wall is out of frame (since it exits the left_target and right_target condition)
                else:
                    left_val = left_x - LEFT_LIMIT
                    right_val = RIGHT_LIMIT - right_x
                    if left_val < right_val:# This condition occurs when the difference in y is less
                        steer(STEER_CENTER + ((right_val - left_val) * STEER_FACTOR_BLACK_DIFF)) #The robot equalises itself between the 2 walls
                    elif right_val < left_val:
                        steer(STEER_CENTER + ((left_val - right_val) * STEER_FACTOR_BLACK_DIFF)) # robot equalises itself between the 2 walls
                    else:
                        steer(STEER_CENTER) # If both walls can be seen and there y values are same, it means we are straight, so keep going straight
            elif left_target:
                only_x, only_y = left_target
                if rounds_completed >12 and CLOCKWISE: 
                # If rounds completed > 12, begin the outer round wall follow. Since we are going clockwise, we simply allign with it keeping some gap.
                    steer(STEER_CENTER + ((only_x - 420) * STEER_FACTOR_SINGLE_TARGET))
                elif rounds_completed >12:
                    steer(STEER_CENTER + ((only_x) * STEER_FACTOR_SINGLE_TARGET))
                """If rounds completed > 12, begin the outer round wall follow. Since we are going anticlockwise, we should be following the right wall, due to this, we make the error as the left wall's x to instantly move away from it."""    
                else:
                    if CLOCKWISE:
         """If rounds completed are less than 12, and are going clockwise and can only see the left wall, we instantly try and correct ourselves in
         order to see both walls."""
                        steer(STEER_CENTER + ((only_x ) * STEER_FACTOR_SINGLE_TARGET))
                    else:
                #If rounds completed are less than 12, and are going anticlockwise and can only see the left wall, we simply allign ourselves with the wall.
                        steer(STEER_CENTER + ((only_x - 80) * STEER_FACTOR_SINGLE_TARGET))
                        
            elif right_target:
                only_x, only_y = right_target
                if rounds_completed > 12 and CLOCKWISE: # CLOCKWISE
                # When we are taking the outer round, we allign ourselevs with the left wall, thus we take the right wall as the error itself
                    steer(STEER_CENTER + ((only_x) * STEER_FACTOR_SINGLE_TARGET))
                elif rounds_completed > 12: #ANTICLOCKWISE
                    steer(STEER_CENTER + ((only_x - 700) * STEER_FACTOR_OUTER_ROUND))
                # When we are taking the outer round, we allign ourselevs with the right wall, thus we maintain a distance with the right wall      
                else:
                    if CLOCKWISE:
                    #If rounds completed are less than 12, and are going anticlockwise and can only see the right wall, we simply allign ourselves with the wall.
                        steer(STEER_CENTER + ((only_x - 1000) * STEER_FACTOR_SINGLE_TARGET))
                    else:
               """If rounds completed are less than 12, and are going anticlockwise and can only see the right wall, we instantly try and correct ourselves in
         order to see both walls."""
                        steer(STEER_CENTER + ((only_x) * STEER_FACTOR_SINGLE_TARGET))
                      
                  
            else:# If nothing can be seen, on the basis of the direction, we can simply turn that way
                if CLOCKWISE:
                    if rounds_completed >12:
                        steer(STEER_CENTER - 10) # Since we are following the outer wall, we turn slightly to allign with the wall
                    else:
                        steer(STEER_CENTER + 15) # Since we have to go inwards, we turn slightly to allign with the direction we are moving in
                else:
                    if rounds_completed >12:
                        steer(STEER_CENTER + 10)# Since we are following the outer wall, we turn slightly to allign with the wall
                    else:
                        steer(STEER_CENTER - 15)  # Since we have to go inwards, we turn slightly to allign with the direction we are moving in
    else:# Begin parking
        reset_heading()
        if not parked and not CLOCKWISE:# If not already parked and direction is anticlockwise
            # Left forward Until angle -82
            steer(78)
            time.sleep(0.5)
            heading=get_heading()
            while get_heading() > -82: # IMU Based. Teams shall have to change (-82) as per their robot. Decrease for sharper turn, increase for less.
                run(90)
                print("turn1, heading: ",heading)
            stop_robot()
            ########################
            # Straight Back Until IR
            steer(110)
            time.sleep(0.35)
            back(70)
            time.sleep(0.7)
            while (GPIO.input(20)==GPIO.HIGH): #Keep reversing robot until Right Back IR Sensor detects the wall
                back(60)
            stop_robot()
            ########################
            # Forward for 0.3 secs
            run(60)
            time.sleep(0.3) # Time based, go slightly forward. Teams shall have to change this as per their IR sensor placement
            stop_robot()
            time.sleep(0.3)
            ########################
            # Backword until Angle -20 
            steer(72)
            time.sleep(0.5)
            while get_heading() < -20: IMU based. Teams may have to change -20 value. Decrease if turn not enough, increase if turn too much
                back(75)
                print("turn2, heading: ",heading)
            stop_robot()
            #########################
            # Forward until angle -5 IMU based. Teams may have to change -5 value. Decrease if turn not enough. Increase if turn too much
            steer(148)
            while get_heading() <= -5:
                run(75)
                print("turn2, heading: ",heading)
            stop_robot()
            parked=True # Set parked to true so that it does not repeat
            steer(110)
            time.sleep(3)      
            
        if not parked and CLOCKWISE:
                    # Left forward Until angle < 80
            steer(142)
            time.sleep(0.5)
            heading=get_heading()
            while get_heading() < 80: # IMU Based. Teams shall have to change (80) as per their robot. Increase for sharper turn, decrease for less sharp.
                run(90)
                print("turn1, heading: ",heading)
            stop_robot()
            ########################
            # Straight Back Until IR Left Back
            steer(110)
            time.sleep(0.35)
            back(70)
            time.sleep(0.7)
            while (GPIO.input(19)==GPIO.HIGH): #Keep reversing robot until Left Back IR Sensor detects the wall
                back(60)
            stop_robot()
            ########################
            # Forward for 0.3 secs
            run(60)
            time.sleep(0.3) # Time based, go slightly forward. Teams shall have to change this as per their IR sensor placement
            stop_robot()
            time.sleep(0.3)
            ########################
            # Backword until Angle 20 
            steer(142)
            time.sleep(0.5)
            while get_heading() > 20 : IMU based. Teams may have to change 20 value. Increase if turn not enough, decrease if turn too much
                back(75)
                print("turn2, heading: ",heading)
            stop_robot()
            #########################
            # Forward until angle >5 IMU based. Teams may have to change -5 value. Decrease if turn not enough. Increase if turn too much
            steer(78)
            while get_heading() >=5:
                run(75)
                print("turn2, heading: ",heading)
            stop_robot()
            parked=True # Set parked to true so that it does not repeat
            steer(110)
            time.sleep(3)      
           # Since we have to go inwards, we turn slightly to allign with the direction we are moving in

    # Display line count on frame for debugging. Note- Frame rate reduces slightly due to this
    cv2.putText(frame, f"Blue lines: {blue_line_count}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(frame, f"Orange lines: {orange_line_count}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
    cv2.putText(frame, f"Rounds: {rounds_completed}/{ROUNDS_GOAL}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Detection", frame)
    video.write(frame)

def stop_robot():
    stop_motors()
    steer(STEER_CENTER)

# === Main loop ===
start = False
stop = True

try:
    reset_heading()
    while True:
        if pi.read(BUTTON_START) == 1 and stop:# If start button is pressed, set variable start to 'True' and stop to 'False'
            
            start = True
            stop = False
            print("Start pressed")
        if pi.read(BUTTON_STOP) == 1 and start: #If stop button is pressed, set variable start to 'False and stop to 'True'
            start = False
            stop = True
            print("Stop pressed")
            reset_heading() 

        if start:# run the function of wall following, line counting, obstacle avoiding and parking
            process_frame()
        else:
            stop_robot() # stop the robot

        if cv2.waitKey(1) & 0xFF == ord('q'): #if q is pressed, then code may be stopped
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    stop_robot() # set speed to 0
    cv2.destroyAllWindows() #shut camera preview
    video.release() # save the video recorded
    picam2.stop() # stop camera
    pi.stop() # stop motor pins
    GPIO.cleanup() # reset GPIO pin states and values
