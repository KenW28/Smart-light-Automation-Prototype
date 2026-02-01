import UltrasonicSensor
import asyncio
import time
import socket
import RPi.GPIO as GPIO
import busio
import board
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from meross_iot.http_api import MerossHttpClient
from meross_iot.manager import MerossManager

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
ads.gain = 1  # set gain to 1 for full range (±4.096V)
chan = AnalogIn(ads, 0)


# -------------------------------------------------------------------
#Login INFO FOR MEROSS CLOUD
EMAIL = "Your_Email"
PASSWORD = "Your_Password"
BUTTON_PIN = 17
# -------------------------------------------------------------------
# web interace SETUP
# -------------------------------------------------------------------
#1) Socket Creation socket()
HOST = "172.20.10.3"  # The server's hostname or IP address
server_port = 6000  # The port used by the server
# -------------------------------------------------------------------
# GPIO SETUP
# -------------------------------------------------------------------
def setup_gpio():
    # Use BCM numbering (GPIO17, etc.)
    GPIO.setmode(GPIO.BCM)

    # BUTTON_PIN as input, with internal pull-up resistor enabled.
    # So: not pressed -> HIGH, pressed -> LOW (to GND).
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


async def wait_for_button_press():
    last_state = GPIO.input(BUTTON_PIN)

    while True:
        current_state = GPIO.input(BUTTON_PIN)

        # Detect falling edge: HIGH -> LOW
        if last_state == GPIO.HIGH and current_state == GPIO.LOW:
            # Simple debounce delay: wait a bit and return
            await asyncio.sleep(0.05)  # 50 ms
            return

        last_state = current_state
        await asyncio.sleep(0.01)  # 10 ms polling interval



# How many cm closer than the baseline counts as "someone walked by"
TRIGGER_DELTA_CM = 10.0


async def main():
    setup_gpio()
    # 1) Login to Meross cloud
    http_api_client = await MerossHttpClient.async_from_user_password(
        email=EMAIL,
        password=PASSWORD,
        api_base_url="https://iot.meross.com"
    )

    # 2)Create manager using that client
    manager = MerossManager(http_client=http_api_client)
    await manager.async_init()

    # 3)Discover devices
    await manager.async_device_discovery()

    devices = manager.find_devices()
    print("Discovered devices:")
    for d in devices:
        print(f"- {d.name} ({d.type})")

    # 4)Pick the first device that supports async_turn_on/off
    plug = None
    for d in devices:
        if hasattr(d, "async_turn_on") and hasattr(d, "async_turn_off"):
            plug = d
            break

    if plug is None:
        print("No toggleable plug-like device found.")
        manager.close()
        await http_api_client.async_logout()
        UltrasonicSensor.cleanup_ultrasonic()
        return

    print(f"Using device: {plug.name} ({plug.type})")
    await plug.async_update()

    # ----------------------------------------------------------------
    # Get initial baseline distance 5 times and average them
    
    print("Measuring baseline distance...")
    baseline_readings = []

    for _ in range(5):
        d = UltrasonicSensor.initial_distance_measure()
        if d is not None:
            baseline_readings.append(d)
            print(f"  sample: {d:5.1f} cm")
        time.sleep(0.1)
    #If no valid readings, exit
    if not baseline_readings:
        print("Could not get baseline distance (no valid readings).")
        manager.close()
        await http_api_client.async_logout()
        UltrasonicSensor.cleanup_ultrasonic()
        return

    baseline_cm = sum(baseline_readings) / len(baseline_readings)
    print(f"Baseline distance set to: {baseline_cm:5.1f} cm")

    # Track light state in software for now
    LIGHT_ON = True
    LIGHT_OFF = False
    light_state = LIGHT_OFF

    print("\nLight is OFF.")
    print("While light is OFF, sensor will watch for someone entering.")

    try:
        while True:
            # Read photoresistor
            voltage = chan.voltage

            # 1) Update light_state from photoresistor
            if voltage < .4:
                # Light is ON
                if light_state == LIGHT_OFF:
                    message = ("LIGHT_ON").encode() #Message to send to server
                    # We just transitioned from ON -> OFF
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp_socket:
                        tcp_socket.connect((HOST, server_port))
                        tcp_socket.sendall(message)
                    tcp_socket.close()
                    # We just transitioned from OFF -> ON
                    print("Light is ON (detected via photoresistor).")
                light_state = LIGHT_ON
            else:
                # Light is OFF
                if light_state == LIGHT_ON:
                    message = ("LIGHT_OFF").encode() #Message to send to server
                    # We just transitioned from ON -> OFF
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp_socket:
                        tcp_socket.connect((HOST, server_port))
                        tcp_socket.sendall(message)
                    tcp_socket.close()
                    print("Light is OFF (detected via photoresistor).")
                    print("While light is OFF, sensor will watch for someone entering.")
                light_state = LIGHT_OFF

            # 2) Behavior based on light_state
            if light_state == LIGHT_OFF:
                # Room/light off: watch for someone entering with ultrasonic
                dist = UltrasonicSensor.new_distance_measure()

                if dist is not None:
                    print(f"Measured: {dist:5.1f} cm (baseline {baseline_cm:5.1f} cm)")

                    # If object is significantly closer than baseline, trigger
                    if dist < baseline_cm - TRIGGER_DELTA_CM:
                        print("Person detected — turning light ON via smart plug.")
                        await plug.async_turn_on()
                        # After this, the photoresistor should eventually show ON (voltage < 1)
                        # and the next loop will flip light_state to LIGHT_ON.
                else:
                    print("No echo (timeout)")

                # Small delay to avoid hammering the sensor
                await asyncio.sleep(0.1)

            else:  # light_state == LIGHT_ON
                # Light is ON: wait for button press to turn it off
                print("Light is ON — waiting for button press to turn off.")
                await wait_for_button_press()
                print("Button pressed, turning light OFF via smart plug.")
                await plug.async_turn_off()
                # After this, once the light actually turns off, the photoresistor
                # will see voltage >= 1.0 and the loop will set light_state = LIGHT_OFF.
                await asyncio.sleep(2)  # debounce


        

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting...")

    finally:
        manager.close()
        await http_api_client.async_logout()
        UltrasonicSensor.cleanup_ultrasonic()
        print("Cleaned up Meross and GPIO. Bye!")


if __name__ == "__main__":
    asyncio.run(main())
