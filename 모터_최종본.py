import LCD1602
import RPi.GPIO as GPIO
import time
import wiringpi as wire

trig = 21 #D12
echo = 23 #D13

LED1 = 10 #D0
LED2 = 8  #D1
LED3 = 19 #D11

EN1 = 15 #D6
EN2 = 22 #D9
EN3 = 24 #D10

IN1 = 11 #D2
IN2 = 12 #D3
IN3 = 26 #D4 
IN4 = 13 #D5
IN5 = 16 #D7
IN6 = 18 #D8


PRODUCT_NUM = 0
spintime = 0.57


GPIO.setmode(GPIO.BOARD)
wire.wiringPiSetup()
wire.wiringPiSPISetup(0, 1000000)

GPIO.setup(EN1, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(EN2, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(EN3, GPIO.OUT, initial = GPIO.LOW)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(IN5, GPIO.OUT)
GPIO.setup(IN6, GPIO.OUT)

GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

GPIO.setup(LED1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED3, GPIO.OUT, initial=GPIO.LOW)


# led 한꺼번에 제어
def led_high():
    GPIO.output(LED1, GPIO.HIGH)
    GPIO.output(LED2, GPIO.HIGH)
    GPIO.output(LED3, GPIO.HIGH)

def led_low():
    GPIO.output(LED1, GPIO.LOW)
    GPIO.output(LED2, GPIO.LOW)
    GPIO.output(LED3, GPIO.LOW)


# 시작 인삿말
def greeting():
    #LCD1602.clear()
    LCD1602.init(0x27,1)
    LCD1602.write(6,0,'Hi :)')
    LCD1602.write(0,1,'Read Inscription')


# 초음파 센서 거리 감지
def beam():
    while 1:
        greeting() 
        GPIO.output(trig, GPIO.LOW)
        time.sleep(0.5)
        GPIO.output(trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig, GPIO.LOW)
        # 테스트 시 확인용
        while(GPIO.input(echo) == GPIO.LOW):
            print("1")
        start_time= time.time()
        while(GPIO.input(echo) == GPIO.HIGH):
            print("2")
        end_time= time.time();
        distance = (end_time - start_time)*1000000/58.0
        print("distance = %.2f cm" %distance)
        
        if distance < 10:
            choose_num()


# 자판기 상품 선택_LCD 출력
def choose_num():
    led_high()
    LCD1602.clear()
    LCD1602.write(0,1,'Choose a Number!')
    PRODUCT_NUM = int(input('번호를 선택하세요:'))
    led_low()
    # 테스트용 변수
    check_num(PRODUCT_NUM)


# LED 깜빡이기
def blink(ledNum):
    if ledNum == 1:
        GPIO.output(LED1, GPIO.HIGH)
    elif ledNum == 2:
        GPIO.output(LED2, GPIO.HIGH)
    else:
        GPIO.output(LED3, GPIO.HIGH)
        
def beam2():
    GPIO.output(trig, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig, GPIO.LOW)
    while(GPIO.input(echo) == GPIO.LOW):
        print("1")
    start_time= time.time()
    while(GPIO.input(echo) == GPIO.HIGH):
        print("2")
    end_time= time.time()
    distance = (end_time - start_time)*1000000/58.0
    print("distance = %.2f cm" %distance)
    return distance

# 선택 번호 확인 질문
def check_num(PRODUCT_NUM):

    LCD1602.clear()
    ask = 'Your choice: ' + str(PRODUCT_NUM)
    LCD1602.write(1,0,ask)
    
    blink(PRODUCT_NUM)
    # 테스트용 변수
 
    start_time= time.time()
    while 1:
        if int(time.time()-start_time)==5:
            break     
                
        if beam2() < 10:
            LCD1602.clear()
            choose_num()
    
    ending(PRODUCT_NUM)


# 마무리 인삿말
def goodbye():
    LCD1602.clear()
    msg = '                Have a nice day:)'
    
    for i in range(0, len(msg)//2):
        led_high()
        LCD1602.write(0,0,msg)
        msg=msg[1:]
        time.sleep(0.3)
        LCD1602.clear()
        led_low()
        LCD1602.write(0,0,msg)
        msg=msg[1:]
        time.sleep(0.3)
        LCD1602.clear()


# 모터 선택
def motorControl(motorNum):
    if motorNum == 1:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(EN1, GPIO.HIGH)
        time.sleep(spintime)
        GPIO.output(EN1, GPIO.LOW)
    elif motorNum == 2:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(EN2, GPIO.HIGH)
        time.sleep(spintime)
        GPIO.output(EN2, GPIO.LOW)
    elif motorNum == 3:
        GPIO.output(IN5, GPIO.HIGH)
        GPIO.output(IN6, GPIO.LOW)
        GPIO.output(EN3, GPIO.HIGH)
        time.sleep(spintime)
        GPIO.output(EN3, GPIO.LOW)
    else:
        choose_num()


# 모터 회전
def ending(PRODUCT_NUM):
    motorControl(PRODUCT_NUM)
    goodbye()
    beam()


def loop():
    beam()
    
            
loop()
GPIO.close()
GPIO.cleanup()

