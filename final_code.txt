import RPi.GPIO as GPIO
import time
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

GPIO_TRIG = 8
GPIO_ECHO = 12
GPIO_LED = 18
BUZZER_PIN = 22

GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_TRIG, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_LED, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

def beep(duration):
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def detect_distance(image):
    GPIO.output(GPIO_TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIG, GPIO.LOW)

    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        bounce_back_time = time.time()

    pulse_duration = bounce_back_time - start_time
    distance = round(pulse_duration * 17150, 2)

    cv2.putText(image, f"Distance: {distance} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Control LED and buzzer based on distance
    if distance < 20:
        GPIO.output(GPIO_LED, GPIO.HIGH)  # Turn on LED
        beep(0.1)  # Sound the buzzer for 0.1 second

        cv2.rectangle(image, (0, 0), (image.shape[1], image.shape[0]), (0, 0, 255), 3)  # Red rectangle for detection
        cv2.putText(image, 'Object Detected', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    else:
        GPIO.output(GPIO_LED, GPIO.LOW)  # Turn off LED

    cv2.imshow("Stream", image)
    key = cv2.waitKey(1) & 0xFF
    return key

try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        result = detect_distance(frame.array)
        rawCapture.truncate(0)
        if result == ord("q"):
            break

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
    cv2.destroyAllWindows()
