#Servo Control Gaz
# import libary
import RPi.GPIO as GPIO
import time
import sys

GPIO.setmode(GPIO.BCM) # set pin mode broadcom

# variables here so easy to alter
servoCenter = 7.4       # servo netrual 7.5 dutycycle
servoLeft = 11.6        # 11.7 dutycycle
servoRight = 2.8        # 2.8  dutycycle
turnTime = 1.0          #delay required for 90 deg turn adjust
spinTime = 0.5          #delay required for a 90 deg spin
moveForward = 5.0       #time to move one body length forward
moveReverse = 5.0       #time to move one body length backwards

distanceLeft = 0.0      #used in avoidance AI 
distanceCenter = 0.0    #used in avoidance AI
distanceRight = 0.0     #used in avoidance AI

 
#set up servo signal GPIO pins 
servopin = 4    #board 7

#set up ultrasonic sensor signal GPIO pins
trig = 40
echo = 37

#set up motor GPIO pins
motor1A = 24
motor1B = 23    #left motor
motor1E = 25

motor2A = 9
motor2B = 10    #right motor
motor2E = 11

#set motor pins as output pins
GPIO.setup(motor1A, GPIO.OUT)
GPIO.setup(motor1B, GPIO.OUT)
GPIO.setup(motor1E, GPIO.OUT)

GPIO.setup(motor2A, GPIO.OUT)
GPIO.setup(motor2B, GPIO.OUT)
GPIO.setup(motor2E, GPIO.OUT)

#set up servopin as output and pwm signal
GPIO.setup(servopin, GPIO.OUT)
p = GPIO.PWM(servopin, 50) #pwm = 50 hz pin 7
p.start(7.5) # start pwm dutycycle of 7.5
print"Center"

def LeftForward():
    GPIO.output(motor1A,GPIO.HIGH)
    GPIO.output(motor1B,GPIO.LOW)
    GPIO.output(motor1E,GPIO.HIGH)

def RightForward():
    GPIO.output(motor2A,GPIO.HIGH)
    GPIO.output(motor2B,GPIO.LOW)
    GPIO.output(motor2E,GPIO.HIGH)

def LeftReverse():
    GPIO.output(motor1A,GPIO.LOW)
    GPIO.output(motor1B,GPIO.HIGH)
    GPIO.output(motor1E,GPIO.HIGH)

def RightReverse():
    GPIO.output(motor2A,GPIO.LOW)
    GPIO.output(motor2B,GPIO.HIGH)
    GPIO.output(motor2E,GPIO.HIGH)

def LeftStop():
    GPIO.output(motor1A,GPIO.LOW)
    GPIO.output(motor1B,GPIO.LOW)
    GPIO.output(motor1E,GPIO.LOW)

def RightStop():
    GPIO.output(motor2A,GPIO.LOW)
    GPIO.output(motor2B,GPIO.LOW)
    GPIO.output(motor2E,GPIO.LOW)
 
def ServoCenter():
    p.ChangeDutyCycle(servoCenter) #each pulse will send the servo towards the netrual position

def ServoRight():
    p.ChangeDutyCycle(servoRight) # 180deg

def ServoLeft():
    p.ChangeDutyCycle(servoLeft) # 0 deg
    
def SettleUltSensor():
    GPIO.output(trig, False)
    print"Waiting for sensor to settle..."
    time.sleep(2)
    print"ULT Settled"
    
def Forward():
    LeftForward()
    RightForward()

def Reverse():
    LeftRevere()
    RightReverse()

def Stop():
    LeftStop()
    RightStop()

def LeftTurn():
    LeftStop()
    RightForward()
    time.sleep(turnTime)

def RightTurn():
    RightStop()
    LeftForward()
    time.sleep(turnTime)

def SpinLeft():
    LeftReverse()
    RightForward()
    time.sleep(spinTime)
    
def SpinRight():
    RightReverse()
    LeftForward()
    time.sleep(spinTime)

def Scan():
    # get distance to object
    GPIO.output(trig, True) #send ping
    time.sleep(0.00001)
    GPIO.output(trig, False)

    while GPIO.input(echo) == 0: #wait for echo
        ping_start = time.time()

    while GPIO.input(echo) == 1: #hear echo
        ping_end = time.time()

    ping_duration = ping_end - ping_start # calc duration of ping
    distance = ping_duration * 17150
    distance = round(distance, 2)
    print "distance = ", distance) 
    
    return distance
    

def ScanLeft():
    ServoLeft()
    Scan()
    distanceLeft = distance
    print"Left range = ", distanceLeft
    
    return distanceLeft

def ScanRight():
    ServoRight()
    Scan()
    distanceRight = distance
    print"Right range = ", distanceRight
    
    return distanceRight

def ScanCenter():
    ServoCenter()
    Scan()
    distanceCenter = distance
    print"Center range = ", distanceCenter
    
    return distanceCenter

    
    
    
    
    

    


    

 SettleUltSensor()

try:
    while True:
       ScanCenter()
       time.sleep(1)
       
       if distanceCenter > 10:                      #no obstacles ahead go forward
           Forward()
           time.sleep(moveForward)
       else:
           Stop()
           ScanLeft()
           print"Left range = ", distanceLeft
           ScanRight()
           print"Right range = ", distanceRight
           ScanCenter()
           print"Center range = ", distanceCenter
           time.sleep(1)            #delay so you can read message, useful for debugging
           
           if distanceLeft > distanceRight and distanceLeft > 11:
               LeftTurn()
           elif distanceRight > distanceLeft and distanceRight > 11:
               RightTurn()
           else:
               Reverse()
               time.sleep(moveReverse)
               Stop()
               
            
           
       
        

except KeyboardInterrupt:
    p.ChangeDutyCycle(servoCenter) #each pulse will send the servo towards the netrual position
    print"Interrupt return to Center"
    time.sleep(5)
    GPIO.cleanup()
    


#need pulses 0.5 ms 1.5ms 2.5ms
# 0.5 /20 = 2.5%
#1.5 / 20 = 7.5%
#2.5 / 20 = 12.5%
