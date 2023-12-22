import RPi.GPIO as GPIO
import picamera
import os
import time
import subprocess
import thingspeak

#thingspeak data
channel_id="2126245"
write_key="58NHC19PS12WS7YF"
client=thingspeak.Channel(channel_id,write_key)

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
led_pin=22
button_pin=17
trig_pin = 23
echo_pin = 24
GPIO.setwarnings(False)
GPIO.setup(led_pin,GPIO.OUT)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # push button

GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)
max_range = 200

GPIO.output(led_pin,False)
Door_Unlocked = 0
Door_Opened = 0

def scan_bluetooth():
    p = subprocess.Popen(["sudo", "hcitool", "scan"], stdout=subprocess.PIPE)
    output, err = p.communicate()
    devices = output.decode().split("\n")

    for device in devices:
        if "98:B8:BC:44:0C:96" in device:
            print("Bluetooth device detected")
            return True
    print("Bluetooth device not found")
    return False

def activate_camera():
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    folder_path="/home/pi/Desktop/Videos"
    file_path=os.path.join(folder_path,"my_video.h264")
    camera.start_recording(file_path)
    camera.wait_recording(5)
    camera.stop_recording()

# Main loop
try:
    while True:
        # Scan for Bluetooth device
        bluetooth_detected = scan_bluetooth()
        if bluetooth_detected:
            # Activate Bluetooth system
            print("Bluetooth system activated")
            Door_Opened = 1
        else:
            # Deactivate Bluetooth system
            print("Bluetooth system deactivated")
            Door_Opened = 0
        
        # Check if door is open
        if bluetooth_detected:
            Door_Unlocked = 1
            try:
                start_time = time.time()
                while time.time() - start_time < 10:
                    while GPIO.input(button_pin) == GPIO.LOW:
                        GPIO.output(led_pin, GPIO.HIGH)
                        Door_Opened = 1
                        # Update ThingSpeak fields
                        fields = {"field1": 1, "field2": Door_Opened}
                        response = client.update(fields)
                        if response != 200:
                            print("Please wait for 15 sec (Free Account Issues:-<")
            finally:
                GPIO.output(led_pin, 0)
                Door_Opened = 0
                Door_Unlocked = 0
        else:
            print("Door Locked")

        # Send a pulse to the ultrasonic sensor
        GPIO.output(trig_pin, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig_pin, GPIO.LOW)

        #Wait for the echo to be received
        while GPIO.input(echo_pin) == GPIO.LOW:
            pulse_start = time.time()

        while GPIO.input(echo_pin) == GPIO.HIGH:
            pulse_end = time.time()

        # Calculate the distance of the object
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # in centimeters

        if distance <= max_range:
            print("Object within range")
            print("Motion detected, camera activated")
            activate_camera()
        else:
            print("Object out of range")

        time.sleep(0.1)


except KeyboardInterrupt:
    GPIO.cleanup() # Corrected indentation here

