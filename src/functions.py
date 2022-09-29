import imaplib
import cv2
import numpy as np
import math
import serial
import time
import os
import RPi.GPIO as gpio
from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from imutils.video import VideoStream
import argparse
import matplotlib.pyplot as plt

smtpUser = 'ENPM809TRP96@gmail.com'
smtpPass = 'raspberrypi2021'

kp = 10
kd = 100
ki = 0.07
dia = 6.5

ser = serial.Serial('/dev/ttyUSB0', 9600)
XCoordinates = []
YCoordinates = []

# A function which stores the robot's distance travelled and the angle turned
def PathPlot(TravelledDistance):
    angle = UpdatedIMUOrientation()
    if XCoordinates == []:
        X = 0
        Y = 0
        XCoordinates.append(0)
        YCoordinates.append(0)
    else:
        X = XCoordinates[-1]
        Y = YCoordinates[-1]
    Xnew = X + (TravelledDistance*np.cos((angle*np.pi)/180))
    Ynew = Y + (TravelledDistance*np.sin((angle*np.pi)/180))
    XCoordinates.append(Xnew)
    YCoordinates.append(Ynew)

# A function that detects a red coloured arrow and determines the direction
def Arrow():
    _ = cv2.VideoCapture(0)
    Message = "Direction detected"
    time.sleep(0.1)
    lower_limit = np.array([176, 64, 86])
    upper_limit = np.array([255, 255, 255])
    while True:
        d, image = cap.read()
        image = cv2.flip(image, -1)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_limit, upper_limit)
        corners = cv2.goodFeaturesToTrack(mask, 7, 0.4, 10)
        corners = np.int0(corners)

        for corner in corners:
            X, Y = corner.ravel()
            cv2.circle(img, (X, Y), 2, (0, 0, 255), -1)
        Ymin = None
        Ymax = None
        Xmax = None
        Xmin = None
        for x in corners:
            for y in x:
                Ax = y[0]
                Ay = y[1]
                if Xmin == None or Ax < Xmin:
                    Xmin = Ax
                if Xmax == None or Ax > Xmax:
                    Xmax = Ax
                if Ymin == None or Ay < Ymin:
                    Ymin = Ay
                if Ymax == None or Ay > Ymax:
                    Ymax = Ay

        Thresh = Xmin + (Xmax - Xmin) / 2

        if (Xmax - Xmin) > (Ymax - Ymin):
            Pt1 = 0
            Pt2 = 0
            for x in corners:
                for y in x:
                    if y[0] < Thresh:
                        Pt1 += 1
                    else:
                        Pt2 += 1

            if Pt1 > Pt2:
                d = "left"
            elif Pt1 < Pt2:
                d = "right"

            SendEmail(str(Message), _)
            return d

# A function to capture a photo
def ImageCapture(_):
    _.release()
    cap = cv2.VideoCapture(0)
    while True:
        __, frame = cap.read()
    
        ImageTime = datetime.now().strftime('%Y%m%d%H%M%S')

        Loc = f'{ImageTime}.jpg'

        cv2.imwrite(Loc, cv2.flip(frame, -1))

        return ImageTime, Loc

def ReceiveEmail():

    mail = imaplib.IMAP4_SSL('imap.gmail.com')
    mail.login('ENPM809TRP96@gmail.com', 'raspberrypi2021')
    mail.list()

    count = 0

    while count < 60:
        try:

            mail.select("inbox")

            result, data = mail.search(None, '(UNSEEN FROM "ENPM809TS19@gmail.com")')
            ids = data[0]
            id_list = ids.split()
            latest_email_id = id_list[-1]
            result, data = mail.fetch(latest_email_id, "(RFC822)")
            if data is not None:
                print("Email Received...")
                print("Show QR Code")
                return data

        except IndexError:
            time.sleep(0)
            if count < 59:
                count = count + 1
                continue
            else:
                print("Rerun code")
                count = 60

# A function to detect and decode QR Code
def QRCodeDetection():
    command = 'sudo modprobe bcm2835-v4l2'
    os.system(command)

    cap = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()
    Message = "QR Code Scanned"
    while True:
        check, img = cap.read()
        try:
            data, bbox, _ = detector.detectAndDecode(img)
        except:
            continue

        if (bbox is not None):
            for i in range(len(bbox)):
                cv2.line(img, tuple(bbox[i][0]), tuple(bbox[(i + 1) % len(bbox)][0]), (0, 0, 255), 4)
        else:
            continue

        if data:
            SendEmail(str(Message),cap)
        if data:
            break
    return data

# A function to send email to defined user
def SendEmail(Message, capture):
    
    ImageTime, loc = ImageCapture(capture)

    toAdd = 'ENPM809TS19@gmail.com'
    cc = 'skotasai@umd.edu , panderajan1996@gmail.com'
    fromAdd = smtpUser
    subject = Message + ' at ' + ImageTime
    rec = cc.split(",") + [toAdd]
    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = fromAdd
    msg['To'] = toAdd
    msg['CC'] = cc

    msg.preamble = "Image recorded at " + ImageTime
    body = MIMEText(Message + " by Rajan's Barron at " + ImageTime)
    msg.attach(body)

    fp = open(loc, 'rb')
    img = MIMEImage(fp.read())
    fp.close()
    msg.attach(img)

    s = smtplib.SMTP('smtp.gmail.com', 587)

    s.ehlo()
    s.starttls()
    s.ehlo()

    s.login(smtpUser, smtpPass)
    s.sendmail(fromAdd, rec, msg.as_string())
    s.quit()

# A function to retrieve the IMU Sensor's original orientation
def IMUOrientation():
    count = 0
    while True:
        if(ser.in_waiting > 0):
            count+=1
            line = ser.readline()
            if count > 10:
                line = line.rstrip().lstrip()
                
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                line = float(line)
                return line

# A function to retrieve the IMU Sensor's updated orientation
def UpdatedIMUOrientation():
    NewLine = ser.readline()
    NewLine = NewLine.rstrip().lstrip()
    NewLine = str(NewLine)
    NewLine = NewLine.strip("'")
    NewLine = NewLine.strip("b'")
    try: 
        NewLine = float(NewLine)
    except:
        return UpdatedIMUOrientation()
    return NewLine

# A function to limit the motor rotation
def RotationLimiter(pwm_a,pwm_b,Counts):
    val = 65
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    buttonBR = int(0)
    buttonFL = int(0)

    pwm_a.start(val)
    pwm_b.start(val)
    while True:
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1

        error = counterBR - counterFL

        LeftMotorSpeed, RightMotorSpeed = ErrorManagement(val, error)

        if LeftMotorSpeed > 100:
            LeftMotorSpeed = 100
        if RightMotorSpeed > 100:
            RightMotorSpeed = 100

        if LeftMotorSpeed < 0:
            LeftMotorSpeed = 0
        if RightMotorSpeed < 0:
            RightMotorSpeed = 0

        pwm_a.ChangeDutyCycle(RightMotorSpeed)
        pwm_b.ChangeDutyCycle(LeftMotorSpeed)
        if counterBR >= Counts:
            pwm_a.stop()
        if counterFL >= Counts:
            pwm_b.stop()
        if counterBR >= Counts or counterFL >= Counts:
            gameover()
            break
        TravelledDistance = (counterBR * np.pi * int(dia)) / 19
        return TravelledDistance

# A function to make the motors move the robot in forward direction
def MoveAhead(x):
    Counts = 19 * (x / (np.pi * int(dia)))
    init()
    pwm_a = gpio.PWM(33, 50)
    pwm_b = gpio.PWM(35, 50)
    dist = RotationLimiter(pwm_a, pwm_b, Counts)
    PathPlot(dist)
    gpio.cleanup()

# A function to make the motors move the robot in reverse direction
def MoveBack(x):
    Counts = 19 * (x / (np.pi * int(dia)))
    init()
    pwm_a = gpio.PWM(31, 50)
    pwm_b = gpio.PWM(37, 50)
    dist = RotationLimiter(pwm_a, pwm_b, Counts)
    PathPlot(dist)
    gpio.cleanup()

# A function to make the motors turn the robot in left direction
def TurnLeft(rotate):
    val = 85
    init()
    pwm_a = gpio.PWM(33, 50)
    pwm_b = gpio.PWM(37, 50)
    loc = UpdatedIMUOrientation()
    des_loc = float(loc) - rotate
    if des_loc < 0:
        des_loc = des_loc + 360
    if des_loc-2 > loc or loc > des_loc+2:
        pwm_a.start(val)
        pwm_b.start(val)

    while True:
        new_loc = UpdatedIMUOrientation()
        if new_loc-3 <= des_loc <= new_loc+3:
            pwm_a.stop(0)
            pwm_b.stop(0)
            gameover()
            break
    gpio.cleanup()    

# A function to make the motors turn the robot in right direction
def TurnRight(rotate):
    val = 85
    init()
    pwm_a = gpio.PWM(31, 50)
    pwm_b = gpio.PWM(35, 50)
    loc = UpdatedIMUOrientation()
    des_loc = float(loc) + rotate
    if des_loc > 360:
        des_loc = des_loc - 360
    if des_loc-2 > loc or loc > des_loc+2:
        pwm_a.start(val)
        pwm_b.start(val)
    while True:
        new_loc = UpdatedIMUOrientation()
        if new_loc-3 <= des_loc <= new_loc+3:
            pwm_a.stop(0)
            pwm_b.stop(0)
            gameover()
            break
    gpio.cleanup()

# A function to initiate RaspberryPi's pins
def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)

# A function to low RaspberryPi's pins
def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)

# A function to introduce control loop to the motors
def ErrorManagement(val, error):
    err = 0
    ogerror = 0
    err = error + err
    LeftMotorSpeed = val - (kp * error + kd * (ogerror - error) + ki * err)
    ogerror = error
    err = error + err
    RightMotorSpeed = val + (kp * error + kd * (ogerror - error) + ki * err)
    ogerror = error
    return LeftMotorSpeed, RightMotorSpeed

# A function to measure and average 10 distances
def DistanceMeasured():
    def dist():
        inf = math.inf
        gpio.setmode(gpio.BOARD)
        gpio.setup(16, gpio.OUT)
        gpio.setup(18, gpio.IN)
    
        gpio.output(16, False)
        time.sleep(0.01)
    
        gpio.output(16, True)
        time.sleep(0.00001)
        gpio.output(16, False)
    
        while gpio.input(18) == 0:
            pulse_start = time.time()
    
        while gpio.input(18) == 1:
            pulse_end = time.time()
        try:
            pulse_duration = pulse_end - pulse_start
        except UnboundLocalError:
            pulse_duration = 1
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        if pulse_duration == 1:
            distance = inf
        return distance
    sum = 0
    for i in range(10):
        a = dist()
        sum = sum + a
    Average = sum / 10
    return Average

# A function to open the gripper
def VialDrop(tf):
    gpio.setmode(gpio.BOARD)
    gpio.setup(36, gpio.OUT)
    pwm = gpio.PWM(36, 50)
    pwm.start(7)
    pwm.ChangeDutyCycle(13)
    time.sleep(tf)
    pwm.stop()
    gpio.output(36, False)
    gpio.cleanup()

# A function to close the gripper
def VialPickUp(tf):
    gpio.setmode(gpio.BOARD)
    gpio.setup(36, gpio.OUT)
    pwm = gpio.PWM(36, 50)
    pwm.start(7)
    pwm.ChangeDutyCycle(7)
    time.sleep(tf)
    pwm.stop()
    gpio.output(36, False)
    gpio.cleanup()

# A function to move the robot from Cold Storage to Injection Area
def TowardsInjectionArea():
    MoveAhead(75)
    TurnLeft(90)
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(110)
    TurnRight(100)
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(110)
   
# A function to move the robot from Injection Area to Cold Storage
def TowardsColdStorage():
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(100)
    while True:
        dist = DistanceMeasured()
        if dist - 27 > 3:
            MoveAhead(1)
        elif dist - 27 < -3:
            MoveBack(1)
        elif 31 > dist > 23:
            break

    dirQR = QRCodeDetection()
    if dirQR == "left":
        TurnLeft(90)
    elif dirQR == "right":
        TurnRight(90)
    else:
        print("Incorrect Location")
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(50)
    while True:
        dist = DistanceMeasured()
        if dist <= 25:
            continue
        else:
            break
    MoveAhead(80)

    while True:
        dist = DistanceMeasured()
        if dist - 40 > 3:
            MoveAhead(1)
        elif dist - 40 < -3:
            MoveBack(1)
        elif 44 > dist > 36 : 
            break
    dirArr=Arrow()
    MoveAhead(20)
    if dirArr == "left":
        MoveAhead(5)
        TurnLeft(90)
    elif dirArr == "right":
        MoveAhead(5)
        TurnRight(90)
    else:
        print("Re-position Arrow")
    MoveAhead(70)

# A function to retrieve the right vial
def Retrieval(lower_limit,upper_limit):
    cap = cv2.VideoCapture(0)
    time.sleep(0.1)

    Message = "Vial picked"
    skip = 0
    while True:
        _, frame = cap.read()
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (640, 480), fx=0, fy=0, interpolation=cv2.INTER_AREA)

        if (skip % 15) != 0:
            skip += 1
            continue
        skip = 1
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_limit, upper_limit)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]

        try:
            cont = contours[0]
        except:
            print('No contours detected')
            continue

        (x, y), radius = cv2.minEnclosingCircle(cont)

        center = (int(x), int(y))
        radius = int(radius)
        diff = int(x) - (640 / 2)
        rotation = (abs(diff) * 0.061)

        if radius < 70:
            VialDrop(1)
            MoveAhead(10)
            VialDrop(1)
        elif radius < 175:
            VialDrop(1)
            MoveAhead(1)
            VialDrop(1)
        else:
            VialPickUp(1)
            cap.release()
            SendEmail(str(Message), cap)
            TurnRight(180)
            MoveAhead(41)
            break

        if radius < 145:
            if diff > 40:
                TurnRight(rotation)
            elif diff < -40:
                TurnLeft(rotation)

# A function call and complete the entire Grand Challenge
def GrandChallenge(st_orient):
    Message = "Vial delivered"

    data = ReceiveEmail()
    if data is not None:
        print("Email received...")
        VialDrop(1)
    VialDrop(1)
    vacc = QRDetection()
    if vacc == "MODERNA":
        lower_limit = np.array([37, 74, 15])
        upper_limit = np.array([105, 255, 255])
    elif vacc == "PFIZER":
        lower_limit = np.array([0, 136, 105])
        upper_limit = np.array([255, 255, 255])
    elif vacc == "J&J":
        lower_limit = np.array([80, 102, 9])
        upper_limit = np.array([130, 255, 255])
    VialPickUp(1)
    VialDrop(1)
    Retrieval(lower_limit, upper_limit)
    fn_orient = UpdatedIMUOrientation()
    st_orient2 = st_orient + 180
    if st_orient2 > 360:
        st_orient2 = st_orient2 - 360
    diff_orient = fn_orient - st_orient2
    if fn_orient > st_orient2:
        TurnLeft(abs(diff_orient))
    elif fn_orient < st_orient2:
        TurnRight(abs(diff_orient))
    else:
        print("Beginning Delivery...")

    TowardsInjectionArea()
    VialDrop(1.5)
    MoveBack(5)
    cap = cv2.VideoCapture(0)
    SendEmail(str(Message), cap)
    TurnRight(180)
    TowardsColdStorage()
    fn_orient2 = UpdatedIMUOrientation()

    if fn_orient2 > 290:
        TurnRight(360 - fn_orient2)
    elif fn_orient2 < 70:
        TurnLeft(fn_orient2 - st_orient)

    while True:

        dist = DistanceMeasured()

        if dist - 58 > 3:
            MoveAhead(1)
        elif dist - 58 < -3:
            MoveBack(1)
        elif 61 > dist > 55:
            break