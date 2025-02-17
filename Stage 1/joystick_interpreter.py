import pygame
import serial
import time

# Initialize the joystick and serial port
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set up serial communication
serial_port = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)  # Allow the serial port to initialize

def map_axis(value, from_min, from_max, to_min, to_max):
    return int((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min)

try:
    while True:
        pygame.event.pump()

        x_axis = map_axis(joystick.get_axis(0), -1.0, 1.0, -100, 100)
        y_axis = map_axis(joystick.get_axis(1), -1.0, 1.0, -100, 100)
        z_axis = map_axis(joystick.get_axis(2), -1.0, 1.0, -100, 100)

        slider = map_axis(joystick.get_axis(3), -1.0, 1.0, 0, 100)

        # tuple of x and y
        hat_x, hat_y = joystick.get_hat(0)

        # Send all values as a single string: "x:y:z:slider:xhat:yhat"
        serial_data = f"{x_axis}:{y_axis}:{z_axis}:{slider}:{hat_x}:{hat_y}\n"
        serial_port.write(serial_data.encode('utf-8'))

        print(f"Sent: {serial_data.strip()}")
        time.sleep(0.1)
finally:
    serial_port.close()
    pygame.quit()
