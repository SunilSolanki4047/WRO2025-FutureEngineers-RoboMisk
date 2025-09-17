
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
button_state=None
offset = 0
prev_heading = None
total_heaeding = 0
# === Pin and hardware config ===
MOTOR_PIN_1 = 24
MOTOR_PIN_2 = 23
MOTOR_ENA = 18
SERVO_PIN = 25
BUTTON_START = 14
BUTTON_STOP = 15
total_heading=0
ParkOver=False
# Motor PWM frequency
MOTOR_PWM_FREQ = 1000
BUTTON_PIN = 26  # Change to the GPIO pin you connected the button to

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(12,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(21,GPIO.IN)
GPIO.setup(20,GPIO.IN)
# Speed settings (0-255 PWM duty cycle)
SPEED_RUN = 80 # 90
SPEED_RUN_SLOW = 75
SPEED_STOP = 0

# Steering angles (degrees)
STEER_CENTER = 110
STEER_LEFT_LIMIT = 78
STEER_RIGHT_LIMIT = 142

# Steering adjustment factors (tuning)
STEER_FACTOR_GREEN_HIGH =0.09
STEER_FACTOR_GREEN_LOW = 0.07
STEER_FACTOR_RED_HIGH = 0.09
STEER_FACTOR_RED_LOW = 0.07
STEER_FACTOR_BLACK_DIFF = 0.07
STEER_FACTOR_SINGLE_TARGET = 0.08
STEER_FACTOR_OUTER_ROUND=0.04
STEER_FACTOR_LEFT_Y_DIFF = 0.07


# STEER_FACTOR_GREEN_HIGH=0.07
# STEER_FACTOR_GREEN_LOW = 0.07
# STEER_FACTOR_RED_HIGH = 0.06
# STEER_FACTOR_RED_LOW = 0.07
# STEER_FACTOR_BLACK_DIFF = 0.08
# STEER_FACTOR_SINGLE_TARGET = 0.04
# STEER_FACTOR_LEFT_Y_DIFF = 0.07

# Camera settings
EXPOSURE_TIME =13000
X_RESOL = 1080
Y_RESOL = 300
X_MID = X_RESOL / 2
LEFT_LIMIT = 10
RIGHT_LIMIT = X_RESOL - 10

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
# Line detection parameters
LINE_COOLDOWN = 3.5  # seconds cooldown before recounting same line
LINE_DETECT_REGION_Y =Y_RESOL-50  # y coordinate near bottom to detect line crossing

# Stop timer duration after rounds complete (seconds)
STOP_TIMER_DURATION = 3
CamOn=True
# Number of rounds robot must complete before stopping
ROUNDS_GOAL = 14

ButtonTime=0

# Line counting mode: 'blue', 'orange', or 'both'
LINE_COUNT_MODE = 'both'  # Change this to 'blue' or 'orange' as needed

# === Initialize PiCamera2 ===
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (X_RESOL, Y_RESOL)})
picam2.preview_configuration.controls.FrameRate = 60
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
pi.set_mode(BUTTON_START, pigpio.INPUT)
pi.set_pull_up_down(BUTTON_START, pigpio.PUD_UP)
pi.set_mode(BUTTON_STOP, pigpio.INPUT)
pi.set_pull_up_down(BUTTON_STOP, pigpio.PUD_UP)

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
        if current_heading is None:
            return total_heading

        if prev_heading is None:
            prev_heading = current_heading
            return total_heading

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

# === Global variables for line counts and cooldown ===
blue_line_count = 0
orange_line_count = 0
last_blue_time = 0
last_orange_time = 0

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
# === Main processing function ===
def process_frame():
    global ButtonTime, Reset, STEER_FACTOR_OUTER_ROUND
    global blue_line_count, orange_line_count, last_blue_time, last_orange_time,purple_target
    global rounds_completed, stop_start_time, stop_motor_done,selected,CLOCKWISE
    global targ1,targ2,insidePark,targ3,targ4,parked,over
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(hsv, (5, 5), 0)
    current_time = time.time()
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

    # Blue line detection
    global ParkOver
    blue_detected = False
    blueDone=0
    if LINE_COUNT_MODE in ('blue', 'both'):
        contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_blue:
            if cv2.contourArea(cnt) > 1000 and ParkOver:
                x, y, w, h = cv2.boundingRect(cnt)
                if y + h >= LINE_DETECT_REGION_Y:
                    blue_detected = True
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    break

    # Orange line detection
    orange_detected = False
    orangeDone=0
    if LINE_COUNT_MODE in ('orange', 'both'):
        contours_orange, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_orange:
            if cv2.contourArea(cnt) > 1000 and ParkOver:
                x, y, w, h = cv2.boundingRect(cnt)
                if y + h >= LINE_DETECT_REGION_Y:
                    orange_detected = True
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 140, 255), 2)
                    break

    if blue_detected and (current_time - last_blue_time) > LINE_COOLDOWN:        
        if GPIO.input(26)==GPIO.LOW or GPIO.input(12)==GPIO.LOW:
            blue_line_count-=1
        blue_line_count += 1
        last_blue_time = current_time
        print(f"Blue line crossed! Count: {blue_line_count}")

    if orange_detected and (current_time - last_orange_time) > LINE_COOLDOWN:       
        if GPIO.input(26)==GPIO.LOW or GPIO.input(12)==GPIO.LOW:
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
        rounds_completed = max(blue_line_count, orange_line_count)

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
                    CLOCKWISE = True
                else:
                    CLOCKWISE = False
                selected = True
            if CLOCKWISE:
                if cx <= X_MID + 10 and cy > max_left_y: # 100
                    max_left_y = cy
                    left_target = (x + w, y + h)
                elif cx > X_MID + 10 and cy > max_right_y: # 100
                    max_right_y = cy
                    right_target = (x, y + h)
            else:
                if cx >= X_MID - 10 and cy > max_right_y: # -100
                    max_right_y = cy
                    right_target = (x, y + h)
                elif cx < X_MID - 10 and cy > max_left_y: # -100
                    max_left_y = cy
                    left_target = (x + w, y + h) # 
                
    # Detect green obstacles
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_target = None
    green_y = 0
    for c in green_contours:
        area = cv2.contourArea(c)
        if 1000 < area <= 20000:
            x, y, w, h = cv2.boundingRect(c)
            cx = x
            cy = y + h
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"Green Obstacle: {cy}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if CLOCKWISE:
                if cy > green_y:
                    green_y = cy
                    green_target = (cx, cy)
                    if rounds_completed >= 12:
                        right_target = (cx, cy)
            else:
                if cy > green_y:
                    green_y = cy
                    green_target = (cx, cy)
                    if rounds_completed >= 12:
                        left_target = (cx, cy)
            
    # Detect red obstacles
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_target = None
    red_y = 0
    for c in red_contours:
        area = cv2.contourArea(c)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w
            cy = y + h
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"Red Obstacle:{cy}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if CLOCKWISE:
                if cy > red_y:
                    red_y = cy
                    red_target = (cx, cy)
                    if rounds_completed >= 12:
                        right_target = (cx,cy)
            else:
                if cy > red_y :
                    red_y = cy
                    red_target = (cx, cy)
                    if rounds_completed >= 12:
                        left_target = (cx,cy)

    current_time = time.time()
    global button_state
    purple_detected = False
    purple_target = None
    global timestart
    timestart=time.time()
    contours_purple, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_purple:
        x, y, w, h = cv2.boundingRect(cnt)         
        if cv2.contourArea(cnt) > 1000:
            if CLOCKWISE:
                purple_detected = True
                cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 105, 255), 2)
                cv2.circle(frame, (x+w, y), 5, (255, 0, 0), -1)
                left_target = (x+w, y)
                purple_target = (x+w, y)
            else:
                purple_detected = True
                cv2.rectangle(frame, (x, y), (x + w, y + h), (180, 105, 255), 2)
                cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)
                right_target = (x, y)
                purple_target = (x, y)
            

    # Update line counts with cooldowns
    # Stop timer handling
    if not over:
        print("Heading : ",abs(get_heading()))
        if insidePark:
            if CLOCKWISE:
                if abs(get_heading()) < 20 and targ1:
                    steer(72)
                    back(85)
                elif abs(get_heading()) < 50 and targ2:
                    targ1 = False
                    steer(148)
                    run(85)
                elif abs(get_heading())>5 and targ3:
                    targ2=False
                    steer(72)
                    run(85)
                else:
                    targ3=False
                    if targ3==False and targ4:
                        run(0)
                        steer(110)
                        time.sleep(0.5)
                        back(100)
                        time.sleep(1)
                        targ4=False
                        insidePark=False
                        time.sleep(0.5)
                        stop_robot()
                        
            else:
                if abs(get_heading()) <15 and targ1:
                    steer(148)
                    back(85)
                
                elif abs(get_heading()) <50 and targ2:
                    targ1 = False
                    steer(72)
                    run(85)
                elif abs(get_heading()) >10 and targ3:
                    targ2=False
                    steer(148)
                    run(85)                                 
                else:
                    targ3=False
                    targ2 = False
                    if targ3==False and targ4:
                        run(0)
                        steer(110)
                        time.sleep(0.5)
                        back(100)
                        time.sleep(1.4)
                        targ4=False
                        insidePark=False
                        run(0)
                        time.sleep(0.45)
                        stop_robot()
                        time.sleep(0.5)
                        ParkOver=True
                        
        elif GPIO.input(12)==GPIO.LOW:
            ButtonTime=time.time()
            run(0)
            time.sleep(0.3)
            steer(142)
            time.sleep(0.4)
            back(90)
            time.sleep(0.5)
            run(0)
            time.sleep(0.4)
            steer(110)
            time.sleep(0.4)
#             back(80)
#             time.sleep(0.4)
#             run(0)
#             time.sleep(0.4)
        elif GPIO.input(26)==GPIO.LOW:
            ButtonTime=time.time()
            run(0)
            time.sleep(0.3)
            steer(78)
            time.sleep(0.4)
            back(90)
            time.sleep(0.5)
            run(0)
            time.sleep(0.4)
            steer(110)
            time.sleep(0.4)
#             back(80)
#             time.sleep(0.4)
#             run(0)
#             time.sleep(0.4)

        elif purple_detected and rounds_completed >= 14:
            run(60)
            only_x, only_y = purple_target
            if CLOCKWISE: # Keep Purple On left when clockwise
                steer(STEER_CENTER + ((only_x - 420) * STEER_FACTOR_SINGLE_TARGET))
            else: #keep purple on right when anticlockwise
                steer(STEER_CENTER + ((only_x - 750) * STEER_FACTOR_SINGLE_TARGET))
            if (GPIO.input(21)==GPIO.LOW):
                over = True
                steer(STEER_CENTER)
                stop_robot()
                time.sleep(2)

                    
        elif green_target and red_target and rounds_completed <=12:
            run(SPEED_RUN)
            if green_y > red_y:
                right_x, right_y = green_target
                if right_y > 100:
                    steer(STEER_CENTER + ((right_x - 820) * STEER_FACTOR_GREEN_HIGH))
                else:
                    steer(STEER_CENTER + ((right_x - 540) * STEER_FACTOR_GREEN_LOW))
            else:
                left_x, left_y = red_target
                if left_y > 100:
                    steer(STEER_CENTER + ((left_x - 260) * STEER_FACTOR_RED_HIGH))
                else:
                    steer(STEER_CENTER + ((left_x - 540) * STEER_FACTOR_RED_LOW))

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

        else:
            run(SPEED_RUN)
            if left_target and right_target:
                left_x, left_y = left_target
                right_x, right_y = right_target
                if left_y - right_y > 80 and not rounds_completed>12:
                    steer(STEER_CENTER + (left_x * STEER_FACTOR_LEFT_Y_DIFF))
                elif right_y - left_y > 80 and not rounds_completed>12:
                    steer(STEER_CENTER + (right_x * STEER_FACTOR_LEFT_Y_DIFF))
                else:
                    left_val = left_x - LEFT_LIMIT
                    right_val = RIGHT_LIMIT - right_x
                    if left_val < right_val:
                        steer(STEER_CENTER + ((right_val - left_val) * STEER_FACTOR_BLACK_DIFF))
                    elif right_val < left_val:
                        steer(STEER_CENTER + ((left_val - right_val) * STEER_FACTOR_BLACK_DIFF))
                    else:
                        steer(STEER_CENTER)
            elif left_target:
                only_x, only_y = left_target
                if rounds_completed >12 and CLOCKWISE:
                    steer(STEER_CENTER + ((only_x - 420) * STEER_FACTOR_SINGLE_TARGET))
                elif rounds_completed >12:
                    steer(STEER_CENTER + ((only_x) * STEER_FACTOR_SINGLE_TARGET))
                else:
                    if CLOCKWISE:
                        steer(STEER_CENTER + ((only_x ) * STEER_FACTOR_SINGLE_TARGET))
                    else:
                        steer(STEER_CENTER + ((only_x - 80) * STEER_FACTOR_SINGLE_TARGET))
            elif right_target:
                only_x, only_y = right_target
                if rounds_completed > 12 and CLOCKWISE:
                    steer(STEER_CENTER + ((only_x) * STEER_FACTOR_SINGLE_TARGET))
                elif rounds_completed > 12:
                    steer(STEER_CENTER + ((only_x - 700) * STEER_FACTOR_OUTER_ROUND))
                else:
                    if CLOCKWISE:
                        steer(STEER_CENTER + ((only_x - 1000) * STEER_FACTOR_SINGLE_TARGET))
                    else:
                        steer(STEER_CENTER + ((only_x - 1080) * STEER_FACTOR_SINGLE_TARGET))
            else:
                if CLOCKWISE:
                    if rounds_completed >12:
                        steer(STEER_CENTER - 15)
                    else:
                        steer(STEER_CENTER + 15)
                else:
                    if rounds_completed >12:
                        steer(STEER_CENTER + 15)
                    else:
                        steer(STEER_CENTER - 15)
    else:
        reset_heading()
        if not parked and not CLOCKWISE:
            # Left forward Until angle -89
            steer(78)
            time.sleep(0.5)
            heading=get_heading()
            while get_heading() > -82:
                run(90)
                print("turn1, heading: ",heading)
            stop_robot()
            ########################
            # Straight Back Until IR
            steer(110)
            time.sleep(0.35)
            back(70)
            time.sleep(0.7)
            while (GPIO.input(20)==GPIO.HIGH):
                back(60)
            stop_robot()
            ########################
            # Forward for 0.3 secs
            run(60)
            time.sleep(0.3)
            stop_robot()
            time.sleep(0.3)
            ########################
            # Backword until Angle -20
            steer(72)
            time.sleep(0.5)
            while get_heading() < -20:
                back(75)
                print("turn2, heading: ",heading)
            stop_robot()
            #########################
            # Forward until angle -5
            steer(148)
            while get_heading() <= -5:
                run(75)
                print("turn2, heading: ",heading)
            stop_robot()
            parked=True
            steer(110)
            time.sleep(3)      
            
        if not parked and CLOCKWISE:
            run(0)
            steer(110)
            time.sleep(1)
            run(70)
            time.sleep(0.7)
            run(0)
            time.sleep(0.4)
            reset_heading()
            print("Headingssss : ",get_heading())
            time.sleep(0.4)
            while count<=3:
                heading=abs(get_heading())
                steer(78)
                time.sleep(0.35)
                while heading<turn1:
                    heading=abs(get_heading())
                    back(65)
                    print("turn1, heading: ",heading)
                run(0)
                steer(142)
                time.sleep(0.35)
                heading=abs(get_heading())
                while (GPIO.input(21)==GPIO.HIGH) or (heading>turn2):
                    heading=abs(get_heading())
                    back(65)
                    print("turn2, heading: ",heading)
                run(0)
                time.sleep(0.5)
                steer(110)
                time.sleep(1.1)
                run(65)
                time.sleep(0.5)
                run(0)
                
                if turn1<=22:
                    turn1=22
                else:
                    turn1=turn1-26
                
                count+=1
            parked =True
            stop_motors()
            time.sleep(10)

#             run(0)
#             steer(110)
#             time.sleep(1)
#             run(55)
#             time.sleep(1.25)
#             #1. Stop then first right turn
#             run(0)
#             if CLOCKWISE:
#                 steer(78)
#             else:
#                 steer(142)
#             time.sleep(1)
#             heading = abs(get_heading())
#             print("Heading: ",heading)
#             time.sleep(1)
#             while heading < 52:
#                 heading = abs(get_heading())
#                 print("Heading in loop 1 : ",heading)
#                 back(75) 
#             # Stop and ready for Left
#             run(0)
#             if CLOCKWISE:
#                 steer(142)
#             else:
#                 steer(78)
#             heading = abs(get_heading())
#             print("Heading Bloop 2",heading)
#             time.sleep(1)
#             #2. Second left turn
#             while heading > 10:
#                 heading = abs(get_heading())
#                 print("Heading in loop 3: ",heading)
#                 back(75) 
#             # Stop and ready for right
#             run(0)
#             steer(110)
#             time.sleep(1)
#             
#             #steer(78)
#             heading = abs(get_heading())
#             print("Heading BLoop", heading)
#             time.sleep(1)
#     #             while heading < -45:
#     #                 heading = getHeading()
#     #                 print("Heading in loop: ",heading)
#     #                 runRev(130) 
#             run(0)
#             if CLOCKWISE:
#                 steer(95)
#             else:
#                 time.sleep(1)
#                 steer(142)
#                 time.sleep(1)
#                 heading=abs(get_heading())
#                 print("starting 3rd turn, heading is: ",abs(get_heading()))
#                 while heading<15:
#                     heading=abs(get_heading())
#                     run(75)
#                 run(0)
#                 steer(78)
#                 time.sleep(1)
#                 heading=abs(get_heading())
#                 while heading>5:
#                     heading=abs(get_heading())
#                     print("starting 4th turn, heading is: ",abs(get_heading()))
#                     back(75)
#                 run(0)
#                 steer(125)
#                 time.sleep(1)
#             # Reverse left
#             #---------
#             run(70)
#             time.sleep(0.3)
#             run(0)
#             steer(110)
#             time.sleep(1)
#     #             runRev(130)
#     #             time.sleep(0.2)
#             run(0)
#             
#             
#     #             steer(78)
#     #             time.sleep(1)
#     #             runRev(130)
#     #             time.sleep(0.1)
#             #-------
#             # Stop and ready for left 
#     #             run(0)
#     #             steer(78)
#     #             time.sleep(1.25)
#     #             # last left turn
#     #             run(130)
#     #             time.sleep(0.25)
#     #             # Stop and Straight
#             run(0)
#             steer(110)
#             time.sleep(1)

    # Display info on frame for debugging
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
        if pi.read(BUTTON_START) == 1 and stop:
            
            start = True
            stop = False
            print("Start pressed")
        if pi.read(BUTTON_STOP) == 1 and start:
            start = False
            stop = True
            print("Stop pressed")
            reset_heading()

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
    GPIO.cleanup()
