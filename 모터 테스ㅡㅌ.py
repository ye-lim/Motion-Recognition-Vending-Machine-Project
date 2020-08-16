import RPi.GPIO as GPIO
import time
import wiringpi as wire

EN2 = 22 #D9
EN3 = 24 #D10

IN1 = 11 #D2
IN2 = 12 #D3
IN3 = 26 #D4 
IN4 = 13 #D5

GPIO.setmode(GPIO.BOARD)
wire.wiringPiSetup()
wire.wiringPiSPISetup(0, 1000000)

GPIO.setup(EN2, GPIO.OUT, initial = GPIO.LOW)

GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)


GPIO.output(IN3, GPIO.HIGH)
GPIO.output(IN4, GPIO.LOW)
GPIO.output(EN2, GPIO.HIGH)
time.sleep(0.57)
GPIO.output(EN2, GPIO.LOW)