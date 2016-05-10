#gaz use Python 3:
#For a robot with two motors and one ULT sersor mounted on one servo
import RPi.GPIO as GPIO  #setting up required libraurys
from time import sleep
from time import time
import sys

GPIO.setmode(GPIO.BCM)  # set the RPi output pins to broadcom numbering
#GPIO.setwarnings(False)

Echo = 17       #board 11 purple wire ultrasonic sensor signal
servopin = 4    #board 7  green wire servo position signal               
                    #left motor
motor1A = 24    #board 18 orange wire
motor1B = 23    #board 16 yellow wire  
motor1E = 25    #board 22 brown wire     enable pin disables the motor when off
                    #right motor
motor2A = 9     #board 21 Blue wire
motor2B = 10    #board 19 white wire  
motor2E = 11    #board 23 grey wire     enable pin

                #set motor pins as output pins
GPIO.setup(motor1A, GPIO.OUT)
GPIO.setup(motor1B, GPIO.OUT)
GPIO.setup(motor1E, GPIO.OUT)

GPIO.setup(motor2A, GPIO.OUT)
GPIO.setup(motor2B, GPIO.OUT)
GPIO.setup(motor2E, GPIO.OUT)

                #set up servopin as output with PWM signal
GPIO.setup(servopin, GPIO.OUT)
p = GPIO.PWM(servopin, 50)  #pwm = 50 hz pin 7
p.start(7.5)    # start pwm dutycycle of 7.5
#print("Center") bebugging setup test


                #variables all here where they can easily be adjusted
stoppingDistance = 4

servoCenter = 7.4       # servo netrual 7.5 dutycycle
servoLeft = 11.6        # 11.7 dutycycle
servoRight = 2.8        # 2.8  dutycycle

turnTime = 1.0          #delay required for 90 deg turn adjust as required
spinTime = 0.5          #delay required for a 90 deg spin
moveForward = 2         #time to move one body length forward
moveReverse = 5.0       #time to move one body length backwards


#functions that can be called at will

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

def Forward():                      #functions within functions with clear names
    LeftForward()
    RightForward()    

def Reverse():
    LeftReverse()
    RightReverse()
    
def Stop():
    LeftStop()
    RightStop()

def LeftTurn():
    LeftStop()
    RightForward()
   
def RightTurn():
    RightStop()
    LeftForward()    

def SpinLeft():
    LeftReverse()
    RightForward()   
    
def SpinRight():
    RightReverse()
    LeftForward()
    
def RadarAhead():
    p.ChangeDutyCycle(servoCenter) #90deg the netrual position of the servo

def RadarLeft():
    p.ChangeDutyCycle(servoLeft) # 0 deg servo left

def RadarRight():
    p.ChangeDutyCycle(servoRight) # 180deg servo right

def GetRange():                 #use the ultrasonic sensor to find distance
  GPIO.setup(Echo,GPIO.OUT)
  GPIO.output(Echo, 0)
  sleep(0.1)
  # print("Sending Trigger")   debugging test
  GPIO.output(Echo,1)           # radar ping
  sleep(0.00001)
  GPIO.output(Echo,0)
  GPIO.setup(Echo,GPIO.IN)
  while GPIO.input(Echo) == 0:  #listen for echo
    pass
  start = time()
  while GPIO.input(Echo) == 1:  # heard echo
    pass
  stop = time()
  elapsed = stop - start        #calculate distance from speed sound / 2 
  distance = elapsed * 17000    #half of soundwave travel there bounce then half travel back
  return distance  
  
#this is the bit where everything happens
# trial avoidance AI

while True:
   
    RadarAhead()                                #make shure servo with ULT sensor is pointing forwards            
    distance = GetRange()                       #call function to find distance ahead
    print("range %.1f " % distance)             #debug check
    sleep(1)
  
    if distance > stoppingDistance:             #as long as nothing is in the way
        
        print("outside braking Distance %.1f " % distance)
        Forward()                                     # obstacle in front stop robot
        sleep(moveForward)
    elif distance < stoppingDistance:               #if something is in the way
         Stop()
         RadarLeft()                                #check for obstacle on left side
         distanceLeft = GetRange()                  
         print("Range Left %.1f " % distanceLeft)    #debugging test
         sleep(2)
         RadarRight()                               #check for obstacle on right side
         distanceRight = GetRange()
         print("Range Right %.1f " % distanceRight)  #debugging test
         sleep(2)
         if distanceRight > distanceLeft and distanceRight > stoppingDistance: #check which way to turn
             print("Turn right")
             RightTurn()
             sleep(turnTime)
             Stop()
             RadarAhead()
         elif distanceLeft > distanceRight and distanceLeft  > stoppingDistance:
             print("Turn left")
             LeftTurn()
             sleep(turnTime)
             Stop()
             RadarAhead()
         elif distanceLeft and distanceRight < stoppingDistance:
             print("Hit a dead end try reverse")
             Reverse()
             sleep(moveReverse)
             Stop()
             RadarAhead()
         else:
             Forward()
    
    else:
        Stop()
        print("I have stopped")
    

    
   
#except KeyboardInterrupt:
#    print("keyboard cntrl + C exit stopping")
 #   Stop()
  #  GPIO.cleanup()
