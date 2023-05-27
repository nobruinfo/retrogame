from evdev import UInput, ecodes as e # Tasten senden
import RPi.GPIO as GPIO # IO lesen

ui = UInput()

# accepts only KEY_* events by default
ui.write(e.EV_KEY, e.KEY_A, 1)  # KEY_A down
ui.write(e.EV_KEY, e.KEY_A, 0)  # KEY_A up
ui.syn()

ui.close()

# Test mit Eingaengen:
GPIO.setmode(GPIO.BCM)
# GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_UP)
# GPIO.setup(17, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(22, GPIO.OUT)
GPIO.output(22, GPIO.LOW)
GPIO.setup(23, GPIO.IN, pull_up_down = GPIO.PUD_UP)
