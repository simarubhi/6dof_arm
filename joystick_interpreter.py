import pygame
import serial
import time

# Initialize the joystick and serial port
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set up serial communication
serial_port = serial.Serial('COM7', 115200, timeout=1)
time.sleep(2)  # Allow the serial port to initialize

def map_axis_to_angle(axis_value):
    """
    Map joystick axis value (-1.0 to 1.0) to a servo adjustment between 10 and -10.
    """
    # Map the axis value from [-1.0, 1.0] to the range [10, -10]
    return int(axis_value * 10)  # Scale the value to the range [-10, 10]

try:
    while True:
        pygame.event.pump()  # Update Pygame's internal event system
        
        # Get the X-axis of the joystick (axis 0 for most joysticks)
        x_axis = joystick.get_axis(0)
        angle_adjustment = map_axis_to_angle(x_axis)
        
        # Send the adjustment over serial as a string (always an integer)
        serial_port.write(f"{angle_adjustment}\n".encode('utf-8'))
        
        print(f"Sent: {angle_adjustment}")
        time.sleep(0.1)  # Adjust frequency of updates
finally:
    serial_port.close()
    pygame.quit()
