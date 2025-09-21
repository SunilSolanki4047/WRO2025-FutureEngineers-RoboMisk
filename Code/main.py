from picamera2 import Picamera2
import cv2
import numpy as np
import time
import pigpio
import datetime
import board
import adafruit_bno055
import RPi.GPIO as GPIO
# === I2C & sensors ==
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)
offset = 0
prev_heading = None
# === Pin and hardware config ===
MOTOR_PIN_1 = 24
timerst=0
MOTOR_PIN_2 = 23
MOTOR_ENA = 18
SERVO_PIN = 25

total_heading=0
ParkOver=False
target=True
reseth=False
# Motor PWM frequency
MOTOR_PWM_FREQ = 1000
LEFT_BUTTON_PIN = 26  # Change to the GPIO pin you connected the button to
RIGHT_BUTTON_PIN = 12
rounds_done2=False
START_BUTTON = 14

GPIO.setmode(GPIO.BCM)

GPIO.setup(LEFT_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RIGHT_BUTTON_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(21,GPIO.IN)
GPIO.setup(20,GPIO.IN)
GPIO.setup(19,GPIO.IN)
GPIO.setup(16,GPIO.IN)
# Speed settings (0-255 PWM duty cycle)
SPEED_RUN = 130# 160 # 130
SPEED_RUN_SLOW = 125#155 # 125
SPEED_STOP = 0

# Steering angles (degrees)
STEER_CENTER = 110
STEER_LEFT_LIMIT = 78
STEER_RIGHT_LIMIT = 142

# Steering adjustment factors (tuning)
STEER_FACTOR_GREEN_HIGH =0.09     #For avoiding green
STEER_FACTOR_GREEN_LOW = 0.09      # For going towards green
STEER_FACTOR_RED_HIGH = 0.09    # for avoiding red
STEER_FACTOR_RED_LOW = 0.09     # for going towards red
STEER_FACTOR_BLACK_DIFF = 0.06   #when robot sees both walls but cant align properly
STEER_FACTOR_SINGLE_TARGET = 0.09   # when only 1 wall detected
STEER_FACTOR_OUTER_ROUND=0.05

# Camera settings
EXPOSURE_TIME =12000
X_RESOL = 1080
Y_RESOL = 350
X_MID = X_RESOL / 2
LEFT_LIMIT = 10
RIGHT_LIMIT = X_RESOL - 10

# HSV thresholds for colors
GREEN_LOWER = np.array([50, 80, 40]) # 50, 100, 50
GREEN_UPPER = np.array([70, 255, 255]) # 70, 255, 255

# RED_LOWER1 = np.array([0, 80, 20])
# RED_UPPER1 = np.array([10, 255, 255])
# RED_LOWER2 = np.array([170, 80, 20])
# RED_UPPER2 = np.array([179, 255, 255])

RED_LOWER1 = np.array([0, 120, 70]) # 0, 120, 50 | 0, 90, 70
RED_UPPER1 = np.array([10, 255, 255]) # 10, 255, 255 | 8, 255, 255
RED_LOWER2 = np.array([170, 120, 70]) # 160, 120, 50 | 170, 90, 70
RED_UPPER2 = np.array([180, 255, 255]) # 165, 255, 255 | 179, 255, 255

BLACK_LOWER = np.array([0, 0, 0]) # 0, 0, 0
BLACK_UPPER = np.array([179, 100, 120]) # 179, 255, 80

BLUE_LOWER = np.array([100, 150, 50])
BLUE_UPPER = np.array([130, 255, 255])

ORANGE_LOWER = np.array([10, 150, 100])
ORANGE_UPPER = np.array([25,255, 255])

PURPLE_LOWER = np.array([160, 60, 30]) # 150, 80, 70
PURPLE_UPPER = np.array([175, 255, 255]) # 170, 255, 255        
# Line detection parameters
LINE_COOLDOWN = 3.5  # seconds cooldown before recounting same line
LINE_DETECT_REGION_Y =Y_RESOL-50  # y coordinate near bottom to detect line crossing

# Stop timer duration after rounds complete (seconds)
# Number of rounds robot must complete before stopping
ROUNDS_GOAL = 14

ButtonTime=0

# Line counting mode: 'blue', 'orange', or 'both'
LINE_COUNT_MODE = 'both'  # Change this to 'blue' or 'orange' as needed

# === Initialize PiCamera2 ===
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (X_RESOL, Y_RESOL)})
picam2.preview_configuration.controls.FrameRate = 50
picam2.configure(preview_config)
picam2.set_controls({
    "AeEnable": False,
    "ExposureTime": EXPOSURE_TIME,
    "AnalogueGain": 4.0
})
timestamp = datetime.datetime.now()
video = cv2.VideoWriter(f"output{timestamp}.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 20, (X_RESOL, Y_RESOL))
picam2.start()
time.sleep(1)

# === Setup GPIO pins ===
pi = pigpio.pi()
GPIO.setup(START_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

for pin in (MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_ENA):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)

pi.set_PWM_frequency(MOTOR_ENA, MOTOR_PWM_FREQ)

# === Robot control functions ===
def steer(angle: float):
    """Set servo angle within limits."""
    angle = max(STEER_LEFT_LIMIT, min(STEER_RIGHT_LIMIT, angle))
    pulse = int(500 + (angle / 180) * 2000)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)

def run(speed: int):
    """Run motors forward at speed (0-255)."""
    pi.write(MOTOR_PIN_1, 1)
    pi.write(MOTOR_PIN_2, 0)
    pi.set_PWM_dutycycle(MOTOR_ENA, speed)

def back(speed: int):
    """Run motors backward at speed (0-255)."""
    pi.write(MOTOR_PIN_1, 0)
    pi.write(MOTOR_PIN_2, 1)
    pi.set_PWM_dutycycle(MOTOR_ENA, speed)

def stop_motors():
    """Stop all motors."""
    pi.set_PWM_dutycycle(MOTOR_ENA, 0)
    pi.write(MOTOR_PIN_1, 0)
    pi.write(MOTOR_PIN_2, 0)

def reset_heading():
    global prev_heading,total_heading
    prev_heading = None
    total_heading = 0

def get_heading():
    global total_heading,prev_heading
    try:
        time.sleep(0.03)
        current_heading = sensor.euler[0]
        
        if prev_heading is None:
            prev_heading = current_heading

        delta = current_heading - prev_heading
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        total_heading += delta
        prev_heading = current_heading
        return total_heading
    except Exception as e:
        print("Error reading heading:", e)
        return total_heading
    
def get_red_mask(hsv):
    """Get combined red mask handling hue wrap."""
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    return cv2.bitwise_or(mask1, mask2)

def revRight():
    stop_robot()
    time.sleep(0.35)
    steer(142)
    time.sleep(0.35)
    back(125)
    time.sleep(0.5)
    stop_robot()
    steer(110)
    time.sleep(0.35)
    run(125)
    time.sleep(0.5)
    stop_robot()

def revLeft():
    stop_robot()
    time.sleep(0.35)
    steer(78)
    time.sleep(0.35)
    back(125)
    time.sleep(0.5)
    stop_robot()
    steer(110)
    time.sleep(0.35)
    run(125)
    time.sleep(0.5)
    stop_robot()
# === Global variables for line counts and cooldown ===
blue_line_count = 0
orange_line_count = 0
last_blue_time = 0
last_green_time = 0
last_orange_time = 0
purple_detected_count = 0
green_detected_count = 0
last_purple_time = 0
# Rounds & stopping control
rounds_completed = 0
stop_start_time = None
stop_motor_done = False
insidePark=True
targ1 = True
targ2 = True 
targ3=True
targ4=True
selected = False
CLOCKWISE = True
over = False
parked = False
Reset=False
rounds_done = False
# === Main processing function ===
def process_frame():
    global ButtonTime, Reset, target, reseth, rounds_done, rounds_done2
    global blue_line_count, orange_line_count,last_green_time, last_blue_time, last_orange_time,purple_target,green_detected_count, purple_detected_count, last_purple_time
    global rounds_completed, stop_start_time, stop_motor_done,selected,CLOCKWISE
    global targ1,targ2,insidePark,targ3,targ4,parked,over
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(hsv, (5, 5), 0)
    current_time = time.time()
    black_mask = cv2.inRange(blur, BLACK_LOWER, BLACK_UPPER)
    green_mask = cv2.inRange(blur, GREEN_LOWER, GREEN_UPPER)
    red_mask = get_red_mask(hsv)
    blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    purple_mask = cv2.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER)
    
    kernel_5 = np.ones((5, 5), np.uint8)
    kernel_50 = np.ones((15, 15), np.uint8)

    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel_50)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_DILATE, kernel_50)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel_5)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel_5)

    # Blue line detection
    global ParkOver
    purple_y=0
    current_time = time.time()
    purple_detected = False
    purple_target = None
    global timestart
    timestart=time.time()
    contours_purple, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_purple:
        x, y, w, h = cv2.boundingRect(cnt)         
        if cv2.contourArea(cnt) > 1000:
            if y+h > purple_y:
                purple_y = y+h
                if CLOCKWISE:
                    purple_detected = True
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 105, 255), 2)
                    cv2.circle(frame, (x+w, y), 5, (255, 0, 0), -1)
                    left_target = (x+w, y)
                    purple_target = (x+w, y+h) 
                else:
                    purple_detected = True
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 105, 255), 2)
                    cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)
                    right_target = (x, y)
                    purple_target = (x, y+h)
                    
   
        
#     if purple_detected and (current_time - last_purple_time) > 7:       
#         purple_detected_count += 1
#         last_purple_time = current_time
    
#     if purple_detected_count >= 5 and current_time - last_purple_time >= 1.1 and not rounds_done:
#         rounds_done = True
#     if purple_detected_count>=5:
#         rounds_done2=True

    global stop_start_time, stop_motor_done
    
    # Find black contours for navigation
    contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    left_target = None
    right_target = None
    max_left_y = 0
    max_right_y = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(contour)
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 105, 255), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            if not selected:
                if cx < X_MID:
                    CLOCKWISE = False # True
                else:
                    CLOCKWISE = False 
                selected = True
            if CLOCKWISE:
                if cx <= X_MID  and cy > max_left_y: # 100
                    max_left_y = cy
                    left_target = (x + w, y + h)
                elif cx > X_MID  and cy > max_right_y: # 100
                    max_right_y = cy
                    right_target = (x, y + h)
            else:
                if cx >= X_MID  and cy > max_right_y: # -100
                    max_right_y = cy
                    right_target = (x, y + h)
                elif cx < X_MID  and cy > max_left_y: # -100
                    max_left_y = cy
                    left_target = (x + w, y + h) # 
                
    # Detect green obstacles
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_target = None
    green_y = 0
    green_x = 0
    for c in green_contours:
        area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)
        if area > 1000 :
            if not CLOCKWISE and rounds_done:
                cx = x + w
                cy = x + h
            else:
                cx = x 
                cy = x + h
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"Green Obstacle: {cy}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)       
            
            if cy > green_y:
                green_y = cy
                green_x = cx
                green_target = (cx, cy)
                target=True
    # Detect red obstacles
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_target = None
    red_y = 0
    red_x = 0
    for c in red_contours:
        area = cv2.contourArea(c)
        if area > 1000 and area < 17000:
            x, y, w, h = cv2.boundingRect(c)
            if CLOCKWISE and rounds_done:
                cx = x 
                cy = y + h
            else:
                cx = x + w
                cy = y + h
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"Red Obstacle:{cy}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            if cy > red_y and not purple_target:
                red_y = cy
                red_x = cx
                red_target = (cx, cy)
                target=False
    # Update line counts with cooldowns
    # Stop timer handling
    if green_target and (current_time - last_green_time) > 3:       
        green_detected_count += 1
        last_green_time = current_time
    
    if green_detected_count >= 9 and current_time - last_green_time >= 1.5 and not rounds_done:
        steer(100)
        run(125)
        time.sleep(4)
        run(0)
        time.sleep(0.5)
        back(125)
        time.sleep(0.1)
        run(0)
        steer(142)
        time.sleep(0.5)
        reset_heading()
        
        time.sleep(0.2)
        while get_heading() > -90 :
            back(125)
        stop_robot()
        time.sleep(0.5)
        rounds_done = True
        
    if not over:
        print("Heading : ",get_heading())
        print("Heading : ",get_heading())
        if insidePark:
            if not reseth:
                reset_heading()
                time.sleep(1)
                reseth=True
            if CLOCKWISE:
                steer(78)
                time.sleep(0.5)
                while get_heading() < 30 :
                    back(120)
                stop_robot()
                steer(142)
                time.sleep(0.5)
                while get_heading() < 85 :
                    run(120)
                stop_robot()
                steer(110)
                time.sleep(0.5)
                run(120)
                time.sleep(0.9)
                stop_robot()
                time.sleep(0.4)
                steer(142)
                time.sleep(0.4)
                while get_heading() > 0 :
                    back(110)                                 
                stop_robot()          
                steer(110)
                back(120)
                time.sleep(0.3)
                stop_robot()
                time.sleep(1)
                insidePark=False                        
            else:
                steer(142)
                time.sleep(0.5)
                while get_heading() > -31 :
                    back(120)
                stop_robot()
                steer(78)
                time.sleep(0.5)
                while get_heading() > -85 :
                    run(120)
                stop_robot()
                steer(110)
                time.sleep(0.5)
                run(120)
                time.sleep(0.9)
                stop_robot()
                time.sleep(0.4)
                steer(78)
                time.sleep(0.4)
                while get_heading() < -5 :
                    back(120)                                 
                stop_robot()
                steer(110)
                time.sleep(0.5)
                back(120)
                time.sleep(0.3)
                stop_robot()
                time.sleep(1)
                insidePark=False
       
        elif (GPIO.input(26)==GPIO.LOW or GPIO.input(12)==GPIO.LOW) and (red_target or green_target):
            if red_target:
                rX, rY = red_target
                if rY >= 250 or (CLOCKWISE and left_target and not right_target):
                    revLeft()
                else:
                    revRight()
            elif green_target:
                gX, gY = green_target
                if gY >= 250 or (not CLOCKWISE and not left_target and right_target):
                    revRight()
                else:
                    revLeft()
        elif (GPIO.input(26)==GPIO.LOW or GPIO.input(12)==GPIO.LOW) and CLOCKWISE and left_target and not right_target: # and rounds_done:
            revLeft()
        elif (GPIO.input(26)==GPIO.LOW or GPIO.input(12)==GPIO.LOW) and not CLOCKWISE and not left_target and right_target: # and rounds_done:
            revRight()
        elif (GPIO.input(26)==GPIO.LOW or GPIO.input(12)==GPIO.LOW) and not CLOCKWISE and left_target and right_target: # and rounds_done:
            revLeft()
        
        elif purple_detected and rounds_done:
            run(90)
            only_x, only_y = purple_target
            if CLOCKWISE: # Keep Purple On left when clockwise
                steer(STEER_CENTER + ((only_x - 400) * 0.09))
                if (GPIO.input(16)==GPIO.LOW):
                    over = True
                    steer(STEER_CENTER)
                    stop_robot()
                    time.sleep(2)
                    print("ir")
                if (only_y>270) and rounds_done2:
                    over = True
                    steer(STEER_CENTER)
                    time.sleep(0.7)
                    run(90)
                    time.sleep(0.2)
                    stop_robot()
                    time.sleep(2)
                    print("cam")
            else: #keep purple on right when anticlockwise
                steer(STEER_CENTER + ((only_x - 680) * 0.09))
                if (GPIO.input(20)==GPIO.LOW):
                    over = True
                    steer(STEER_CENTER)
                    stop_robot()
                    time.sleep(2)
                    print("ir")
                if (only_y > 270) and rounds_done2:
                    over = True
                    steer(STEER_CENTER)
                    time.sleep(0.7)
                    run(90)
                    time.sleep(0.2)
                    stop_robot()
                    time.sleep(2)
                    print("cam")
      
        elif green_target and red_target:
            run(SPEED_RUN)
            if green_y > red_y:
                right_x, right_y = green_target
                if right_y > 140:
                    steer(STEER_CENTER + ((right_x - 880) * STEER_FACTOR_GREEN_HIGH))
                else:
                    steer(STEER_CENTER + ((right_x - 640) * STEER_FACTOR_GREEN_LOW))
            else:
                left_x, left_y = red_target
                if left_y > 120:
                    steer(STEER_CENTER + ((left_x - 200) * STEER_FACTOR_RED_HIGH ))
                else:
                    steer(STEER_CENTER + ((left_x - 440) * STEER_FACTOR_RED_LOW))

        elif green_target:
            run(SPEED_RUN)
            if CLOCKWISE:
                right_x, right_y = green_target
                if right_y > 90:
                    steer(STEER_CENTER + ((right_x - 880) * STEER_FACTOR_GREEN_HIGH + 0.04))
                else:
                    steer(STEER_CENTER + ((right_x - 540) * STEER_FACTOR_GREEN_LOW))
            else:
                right_x, right_y = green_target
                if right_y > 110:
                    steer(STEER_CENTER + ((right_x-900) * STEER_FACTOR_GREEN_HIGH))
                    print("920")
                else:
                    print("540")
                    steer(STEER_CENTER + ((right_x - 540) * STEER_FACTOR_GREEN_LOW))
    
        elif red_target:
            run(SPEED_RUN)
            if CLOCKWISE:
                left_x, left_y = red_target
                if left_y > 80:
                    steer(STEER_CENTER+ ((left_x - 175) * STEER_FACTOR_RED_HIGH ))
                else:
                    steer(STEER_CENTER + ((left_x - 440) * STEER_FACTOR_RED_LOW))
            else:
                left_x, left_y = red_target
                if left_y > 50:
                    steer(STEER_CENTER + ((left_x - 80) * STEER_FACTOR_RED_HIGH))
                else:
                    steer(STEER_CENTER + ((left_x - 240) * STEER_FACTOR_RED_LOW))

        else:
            run(SPEED_RUN)
            if left_target and right_target:
                left_x, left_y = left_target
                right_x, right_y = right_target
                
                left_val = left_x - LEFT_LIMIT
                right_val = RIGHT_LIMIT - right_x
                if left_val < right_val:
                    steer(STEER_CENTER - ((right_val - left_val) * STEER_FACTOR_BLACK_DIFF))
                elif right_val < left_val:
                    steer(STEER_CENTER + ((left_val - right_val) * STEER_FACTOR_BLACK_DIFF))
                else:
                    steer(STEER_CENTER)
            elif left_target:
                only_x, only_y = left_target
                if CLOCKWISE and rounds_done:
                    steer(STEER_CENTER + ((only_x - 430) * STEER_FACTOR_OUTER_ROUND))
                elif rounds_done:
                    steer(STEER_CENTER + ((only_x) * STEER_FACTOR_SINGLE_TARGET))
                else:
                    if CLOCKWISE:
                        steer(STEER_CENTER + ((only_x ) * (0.02+STEER_FACTOR_SINGLE_TARGET)))
                    else:
                        steer(STEER_CENTER + ((only_x - 80) * (0.02+STEER_FACTOR_SINGLE_TARGET)))
            elif right_target:
                only_x, only_y = right_target
                if CLOCKWISE and rounds_done:
                    steer(STEER_CENTER + ((only_x - 1080) * (0+STEER_FACTOR_SINGLE_TARGET)))
                elif rounds_done:
                    steer(STEER_CENTER + ((only_x - 750) * (0.02+STEER_FACTOR_OUTER_ROUND)))
                else:
                    if CLOCKWISE:
                        steer(STEER_CENTER + ((only_x - 930) * (0.02+STEER_FACTOR_SINGLE_TARGET)))
                    else:
                        steer(STEER_CENTER + ((only_x - 1080) * (0.02+STEER_FACTOR_SINGLE_TARGET)))
            else:
                if CLOCKWISE:
                    steer(STEER_CENTER + 20)
                else:
                    steer(STEER_CENTER - 20)
    else:
        if not parked and not CLOCKWISE:
            reset_heading()
            time.sleep(1)
            # Left forward Until angle -89
            steer(110)
            time.sleep(0.5)
            run(110)
            time.sleep(0.23)
            stop_robot()
            time.sleep(0.4)
            steer(78)
            time.sleep(0.5)
            heading=get_heading()
            while get_heading() > -45:
                run(110)
                print("turn1, heading: ",heading)
            stop_robot()
            
            
            ########################
            # Straight Back Until IR
            steer(142)
            time.sleep(0.3)
            while get_heading() > -85:                
                back(110)
            stop_robot()
            time.sleep(0.4)
            while (GPIO.input(20)==GPIO.HIGH):
                back(110)
            stop_robot()
            time.sleep(0.5)
            ########################
            # Forward for 0.2 secs
            run(110)
            time.sleep(0.25)
            stop_robot()
            time.sleep(0.5)
            ########################
            # Backword until Angle -20
            steer(78)
            time.sleep(0.5)
            timeA=time.time()
            while get_heading() < -20 and timeA + 2.5 >= time.time():
                 back(110)
                 print("turn2, heading: ",heading)
            stop_robot()
            #########################
            # Forward until angle -5
            steer(142)
            time.sleep(0.5)
            while get_heading() <= -10:
                run(110)
                print("turn3, heading: ",heading)
            stop_robot()
            run(110)
            time.sleep(0.13)
            stop_motors()
            parked=True
            steer(110)
            time.sleep(3)      
            
        if not parked and CLOCKWISE:
            reset_heading()
            time.sleep(1)
            # Left forward Until angle -89
            steer(110)
            time.sleep(0.5)
            run(110)
            time.sleep(0.2)
            stop_robot()
            time.sleep(0.4)
            steer(142)
            time.sleep(0.5)
            heading=get_heading()
            while get_heading() < 45:
                run(125)
            stop_robot()
            ########################
            # Straight Back Until IR
            steer(110)
            time.sleep(0.3)
            steer(78)
            time.sleep(0.5)
            while get_heading()< 83:
                back(115)
            stop_robot()
            time.sleep(0.5)
            while (GPIO.input(16)==GPIO.HIGH):
                back(110)
            stop_robot()
            time.sleep(0.5)
            ########################
            # Forward for 0.2 secs
            run(110)
            time.sleep(0.2)
            stop_robot()
            ########################
            # Backword until Angle -20
            steer(142)
            time.sleep(0.5)
            startt=time.time()
            while get_heading() > 25 and startt + 2.5 >= time.time():
                 back(120)
                 print("turn2, heading: ",heading)
            stop_robot()
            #########################
            # Forward until angle -5
            steer(78)
            time.sleep(0.5)
            while get_heading() > 15:
                run(110)
                print("turn2, heading: ",heading)
            stop_robot()
            run(110)
            time.sleep(0)
            stop_motors()
            parked=True
            steer(110)
            time.sleep(3)
            run(0)    # Display info on frame for debugging)
    cv2.putText(frame, f"park lines: {purple_detected_count}", (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(frame, f"park lines: {green_detected_count}", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv2.imshow("Detection", frame)
    video.write(frame)

def stop_robot():
    stop_motors()
    steer(STEER_CENTER)

# === Main loop ===
start = False
stop = True
pressTime=0
try:
    reset_heading()
    while True:

        if GPIO.input(START_BUTTON) == GPIO.HIGH and start and pressTime + 2 < time.time():
            print("Stop Pressed")
            stop = True
            start = False
            pressTime = time.time()
        if GPIO.input(START_BUTTON) == GPIO.HIGH and stop and pressTime + 2 < time.time():
            print("Start Pressed")
            start = True
            stop = False
            pressTime = time.time()
        
        if start:
            process_frame()
        else:
            stop_robot()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    stop_robot()
    cv2.destroyAllWindows()
    video.release()
    picam2.stop()
    pi.stop()
    GPIO.cleanup();
