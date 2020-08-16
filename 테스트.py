import cv2 as cv
import numpy as np
import os
import time

import LCD1602
import RPi.GPIO as GPIO
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


current_file_path = os.path.dirname(os.path.realpath(__file__))
#cascade = cv.CascadeClassifier(current_file_path + "\haarcascade_frontalface_alt.xml")
cascade = cv.CascadeClassifier(current_file_path + "/haarcascade_frontalface_alt.xml")
#################################################


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
            a=0
        start_time = time.time()
        while(GPIO.input(echo) == GPIO.HIGH):
            a=0
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
      
    # cap = cv.VideoCapture('test.avi')
    cap = cv.VideoCapture(-1)

    start = time.time()
    arr=[-1,-1,-1,-1,-1,-1]
    max1=0
    index=0
    while True:

        if int(time.time()-start)==30:
            break
  
        ret,img_bgr = cap.read()

        if ret == False:
            break

        cnt = process(img_bgr, debug=False)
        arr[cnt]+=1
    
    
            
        key = cv.waitKey(1)
        if key== 27:
            break
    for i in range(6):
        if i==0:
            continue
        if max1<arr[i]:
            max1=arr[i]
            index=i
        
    #  cv.imshow("Result", img_result)

    print(index)
    
    if index==0 or index>=4:
        cap.release()
        cv.destroyAllWindows()
        LCD1602.clear()
        LCD1602.write(0,2,'Try Again..')
        time.sleep(2)
        choose_num()
        
    cap.release()
    cv.destroyAllWindows()
    
    PRODUCT_NUM = index
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
        a=0
    start_time= time.time()
    while(GPIO.input(echo) == GPIO.HIGH):
        a=0
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
    LCD1602.write(5,1,'Right?')
    start_time= time.time()
    while 1:
        if int(time.time()-start_time)==10:
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
    










################################################################
def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=5, minSize=(10, 10),
                                     flags=cv.CASCADE_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects


def removeFaceAra(img, cascade):
  gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
  gray = cv.equalizeHist(gray)
  rects = detect(gray, cascade)

  height,width = img.shape[:2]

  for x1, y1, x2, y2 in rects:
      cv.rectangle(img, (x1-10, 0), (x2+10, height), (0,0,0), -1)

  return img


def make_mask_image(img_bgr):

  img_hsv = cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV)

  #img_h,img_s,img_v = cv.split(img_hsv)

  low = (0, 30, 0)
  high = (15, 255, 255)

  img_mask = cv.inRange(img_hsv, low, high)
  return img_mask


def distanceBetweenTwoPoints(start, end):

  x1,y1 = start
  x2,y2 = end

  return int(np.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)))


def calculateAngle(A, B):

  A_norm = np.linalg.norm(A)
  B_norm = np.linalg.norm(B)
  C = np.dot(A,B)

  angle = np.arccos(C/(A_norm*B_norm))*180/np.pi
  return angle


def findMaxArea(contours):

  max_contour = None
  max_area = -1


  for contour in contours:
    area = cv.contourArea(contour)

    x,y,w,h = cv.boundingRect(contour)

    if (w*h)*0.4 > area:
        continue

    if w > h:
        continue

    if area > max_area:
      max_area = area
      max_contour = contour

  if max_area < 10000:
    max_area = -1

  return max_area, max_contour


def getFingerPosition(max_contour, img_result, debug):
  points1 = []


  # STEP 6-1
  M = cv.moments(max_contour)

  cx = int(M['m10']/M['m00'])
  cy = int(M['m01']/M['m00'])


  max_contour = cv.approxPolyDP(max_contour,0.02*cv.arcLength(max_contour,True),True)
  hull = cv.convexHull(max_contour)

  for point in hull:
    if cy > point[0][1]:
      points1.append(tuple(point[0]))

  if debug:
    cv.drawContours(img_result, [hull], 0, (0,255,0), 2)
    for point in points1:
      cv.circle(img_result, tuple(point), 15, [ 0, 0, 0], -1)


  # STEP 6-2
  hull = cv.convexHull(max_contour, returnPoints=False)
  defects = cv.convexityDefects(max_contour, hull)

  if defects is None:
    return -1,None

  points2=[]
  for i in range(defects.shape[0]):
    s,e,f,d = defects[i, 0]
    start = tuple(max_contour[s][0])
    end = tuple(max_contour[e][0])
    far = tuple(max_contour[f][0])

    angle = calculateAngle( np.array(start) - np.array(far), np.array(end) - np.array(far))

    if angle < 90:
      if start[1] < cy:
        points2.append(start)

      if end[1] < cy:
        points2.append(end)

  if debug:
    cv.drawContours(img_result, [max_contour], 0, (255, 0, 255), 2)
    for point in points2:
      cv.circle(img_result, tuple(point), 20, [ 0, 255, 0], 5)


  # STEP 6-3
  points = points1 + points2
  points = list(set(points))


  # STEP 6-4
  new_points = []
  for p0 in points:

    i = -1
    for index,c0 in enumerate(max_contour):
      c0 = tuple(c0[0])

      if p0 == c0 or distanceBetweenTwoPoints(p0,c0)<20:
        i = index
        break

    if i >= 0:
      pre = i - 1
      if pre < 0:
        pre = max_contour[len(max_contour)-1][0]
      else:
        pre = max_contour[i-1][0]

      next = i + 1
      if next > len(max_contour)-1:
        next = max_contour[0][0]
      else:
        next = max_contour[i+1][0]


      if isinstance(pre, np.ndarray):
            pre = tuple(pre.tolist())
      if isinstance(next, np.ndarray):
        next = tuple(next.tolist())


      angle = calculateAngle( np.array(pre) - np.array(p0), np.array(next) - np.array(p0))

      if angle < 90:
        new_points.append(p0)

  return 1,new_points


def process(img_bgr, debug):

  img_result = img_bgr.copy()

  # STEP 1
  img_bgr = removeFaceAra(img_bgr, cascade)


  # STEP 2
  img_binary = make_mask_image(img_bgr)


  # STEP 3
  kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
  img_binary = cv.morphologyEx(img_binary, cv.MORPH_CLOSE, kernel, 1)
  #cv.imshow("Binary", img_binary)


  # STEP 4
  _, contours, hierarchy = cv.findContours(img_binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

  if debug:
    for cnt in contours:
      cv.drawContours(img_result, [cnt], 0, (255, 0, 0), 3)


  # STEP 5
  max_area, max_contour = findMaxArea(contours)

  if max_area == -1:
      
    cv.imshow("Result", img_result)

    return 0

  if debug:
    cv.drawContours(img_result, [max_contour], 0, (0, 0, 255), 3)


  # STEP 6
  ret,points = getFingerPosition(max_contour, img_result, debug)


  # STEP 7
  a = 0
  if ret > 0 and len(points) > 0:
    for point in points:
      cv.circle(img_result, point, 20, [ 255, 0, 255], 5)
      a += 1
    #print(len(points))

  cv.imshow("Result", img_result)
      
  return a

########################################################





           
loop()
GPIO.close()
GPIO.cleanup()

