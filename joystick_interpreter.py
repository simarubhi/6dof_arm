import pygame
import serial
import time

def map_value(value, in_min, in_max, out_min, out_max): # Map a value from one range to another
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# def apply_deadzone(value, deadzone=20):
#     """Apply a dead zone to axis values."""
#     return 0 if abs(value) <= deadzone else value

def main():
    pygame.init()

    ser = serial.Serial('COM7', 115200)
    time.sleep(2)  # Wait for the serial connection to initialize

    # Initialize joystick
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Initialized joystick: {joystick.get_name()}")

    while True:
        pygame.event.pump()

        # Read joystick axes
        x = joystick.get_axis(0)
        y = joystick.get_axis(1)
        z = joystick.get_axis(2)
        slider = joystick.get_axis(3)  # Slider

        # Convert axis values to range (0 to 100 for slider, -100 to 100 for others)
        x = map_value(x, -1, 1, -100, 100)
        y = map_value(y, -1, 1, -100, 100)
        z = map_value(z, -1, 1, -100, 100)
        slider = map_value(slider, -1, 1, 0, 100)

        # Apply dead zone
        # x = apply_deadzone(x)
        # y = apply_deadzone(y)
        # z = apply_deadzone(z)

        hat = joystick.get_hat(0)  # Returns a tuple (x, y)

        data_to_send = f"{x},{y},{z},{slider},{hat[0]},{hat[1]}\n"

        ser.write(data_to_send.encode('utf-8'))

        data_to_print = f"X:{x:+d} Y:{y:+d} Z:{z:+d} Slider:{slider} Hat:{hat[0]},{hat[1]}"
        print(data_to_print)

        time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
        pygame.quit()
