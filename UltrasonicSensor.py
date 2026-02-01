import RPi.GPIO as GPIO   # Library to control the Raspberry Pi's GPIO pins
import time               # For delays and timing measurements

# -------------------------
# GPIO SETUP
# -------------------------

# Tell RPi.GPIO to use BCM numbering (GPIO23, GPIO24, etc.)
# instead of physical pin numbers (like pin 16, pin 18).
GPIO.setmode(GPIO.BCM)

# We choose which GPIO pins are connected to the ultrasonic sensor:
TRIG_PIN = 23   # GPIO23 -> TRIG pin of the sensor (output from Pi)
ECHO_PIN = 24   # GPIO24 -> ECHO pin of the sensor (input to Pi, via resistor divider)

# Configure TRIG as an OUTPUT: Pi will drive this pin HIGH/LOW to start a measurement
GPIO.setup(TRIG_PIN, GPIO.OUT)

# Configure ECHO as an INPUT: Pi will read this pin to measure pulse length from sensor
GPIO.setup(ECHO_PIN, GPIO.IN)


def initial_distance_measure():
    """
    Send a short pulse on TRIG, then measure how long ECHO stays HIGH.
    Convert that time into a distance in centimeters.
    """

    # 1) Make sure TRIG is LOW for a brief moment to let sensor settle
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.0002)  # 200 microseconds (0.0002 seconds)

    # 2) Send a 10 microsecond HIGH pulse on TRIG to start the measurement
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # 3) Wait for ECHO to go HIGH
    #    This means the sensor has sent a sound pulse and is now timing the echo.
    timeout_start = time.time()  # record the current time
    while GPIO.input(ECHO_PIN) == 0:  # while ECHO is LOW, we are waiting
        # If we wait too long, assume something is wrong (no echo) and abort
        if time.time() - timeout_start > 0.02:  # 20 ms timeout
            return None

    # As soon as ECHO goes HIGH, record that time as the start of the pulse
    start_time = time.time()

    # 4) Now wait for ECHO to go LOW again
    #    While ECHO is HIGH, the sound is "in flight". Pulse width = travel time.
    timeout_start = time.time()
    while GPIO.input(ECHO_PIN) == 1:  # while ECHO is HIGH
        if time.time() - timeout_start > 0.02:  # again, 20 ms timeout
            return None

    # ECHO just went LOW: record that time as the end of the pulse
    end_time = time.time()

    # 5) Calculate how long ECHO was HIGH
    pulse_duration = end_time - start_time  # in seconds

    # 6) Convert time into distance
    #
    # Speed of sound ~ 34300 cm/s (at room temperature).
    # The pulse goes out and comes back, so distance is:
    # (speed of sound * time) / 2
    distance_cm = (pulse_duration * 34300) / 2

    return distance_cm

# get new distance measurements until user stops the program
def new_distance_measure():
    """
    Continuously measure distance until the user stops the program.
    This function is called in a loop to keep getting new measurements.
    """
    new_distance_cm = initial_distance_measure()
    if new_distance_cm is not None:
        return new_distance_cm
    else:
        return None
    




try:
    # Main loop: repeatedly measure and print distance until user hits Ctrl+C
    while True:
        dist = initial_distance_measure()

        if dist is not None:
            # :5.1f = format as fixed-width: 5 chars wide, 1 decimal place
            print(f"Distance: {dist:5.1f} cm")
        else:
            # We hit a timeout, meaning no valid echo was detected this cycle
            print("No echo (timeout)")

        # Wait half a second before the next measurement
        time.sleep(0.5)

except KeyboardInterrupt:
    # This triggers when you press Ctrl+C in the terminal to stop the program
    print("\nStopping...")

finally:
    # Always reset the GPIO pins back to a safe state when exiting
    GPIO.cleanup()
    print("GPIO cleaned up.")
