#Task 7.2D
#This project/code uses an ultrasonic sensor to detect objects between 50cm and 5cm to the sensor
#and translates that information into PWM signals for an actuator
#Physical objects at 50cm will set the actuaor to -90 degrees and as objects get closer to the ultrasonic sensor will bring the actuator closer to +90 degrees
import RPi.GPIO as GPIO
import time, math, sys
from gpiozero import AngularServo #Library for servo motors
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18 # 18(BCM)  12(BOARD)
GPIO_ECHO = 24 #24(BCM)   18(BOARD)
GPIO_PWM = 12 #12(bcm) 32(BOARD)
servo = AngularServo(GPIO_PWM, min_pulse_width=0.0006, max_pulse_width=0.0023)

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT) #//TX trigger
GPIO.setup(GPIO_ECHO, GPIO.IN) #// RX Echo

#Scale Ultrasonic distances to other values for PWM use.  
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    a= math.ceil(rightMin + (valueScaled * rightSpan)) #math.ceil rounds up and returns the smaller integer greater than or equal to a given number.
     
    return a 
 
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
if __name__ == '__main__':
    try: #Get distance, translate distance to cm value, use that value and translate to a PWM actuaor -90 to +90 degrees, send data to actuator
        while True:
            dist = (int(math.ceil(distance()))) #Get rounded up INT value of ultrasonic distances
            if dist>=5 and dist <=50 : #For testing use ultrasonic distances between 0 and 50cm
                x = translate(dist,5,50,90,-90) # mapping the returned distance( 1 to 50cm) into PWM usable values between minus 90 and plus 90 degrees which is returned as an int.
                if x>=-90 and x <=90 : #Make sure we send only good data to the actuator
                    print ("Measured Distance = %f cm and the PWM=%d" % (dist, x) )  #Info of live data values.
                    servo.angle = x #Send the new angle data to the actuator
            time.sleep(.075) #delay a bit !
 
        # Reset by pressing CTRL + c
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
        sys.exit()
