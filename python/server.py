#!/usr/bin/env python3
import pygame
import serial
import time
import sys

# Setup the serial connection parameters
serial_port = '/dev/cu.Dummy_Robot'
baud_rate = 9600
data_bits = serial.EIGHTBITS
parity = serial.PARITY_NONE
stop_bits = serial.STOPBITS_ONE


BTN_A:int = 0
BTN_B:int = 1
BTN_X:int = 2
BTN_Y:int = 3
BTN_L3:int = 7
BTN_R3:int = 8
BTN_L1:int = 9
BTN_R1:int = 10
BTN_RST:int = 6



# Initialize Pygame
pygame.init()
# Initialize the joystick module
pygame.joystick.init()

# Check for attached joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    sys.exit()
else:
    # Create a Joystick object for the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick initialized: {joystick.get_name()}")

try:
    ser = serial.Serial(port=serial_port,
                        baudrate=baud_rate,
                        bytesize=data_bits,
                        parity=parity,
                        stopbits=stop_bits,
                        timeout=1)  # Timeout for read operations

    print(f"Connected to {serial_port} at {baud_rate} baud.")
    running = True
    while running:
        while ser.in_waiting > 0:  # While there is data in the input buffer
          response = ser.readline().decode('utf-8').strip()
          print("Robot: >", response)

        for event in pygame.event.get():
            # Check if the joystick button is pressed
            if event.type == pygame.JOYBUTTONDOWN:
              # print(f"Button {event.button} pressed")
              if event.button == BTN_X:
                  ser.write(b'LED_G\n')
              elif event.button == BTN_Y:
                  ser.write(b'LED_R\n')
              elif event.button == BTN_R3:
                  ser.write(b'READ_DISTANCE\n')
              elif event.button == BTN_A:
                  ser.write(b'DETECT_OBSTACLES\n')
              elif event.button == BTN_B:
                  ser.write(b'BEEP\n')
              elif event.button == BTN_RST:
                  ser.write(b'INIT\n')
              elif event.button == BTN_L1:
                  ser.write(b'TURN_LEFT\n')
              elif event.button == BTN_R1:
                  ser.write(b'TURN_RIGHT\n')

            # Check if the joystick button is released
            if event.type == pygame.JOYBUTTONUP:
              if event.button == BTN_L1 or event.button == BTN_R1:
                  ser.write(b'STOP\n')
            #     print(f"Button {event.button} released")

            # Check for joystick movement
            if event.type == pygame.JOYAXISMOTION:
                # Assuming a joystick with at least two axes
                lx_axis = joystick.get_axis(0)  # X axis
                ly_axis = joystick.get_axis(1)  # Y axis
                rx_axis = joystick.get_axis(2)  # Y axis
                ry_axis = joystick.get_axis(3)  # Y axis
                # print(f"LX axis: {lx_axis:.2f}, LY axis: {ly_axis:.2f} RX axis: {rx_axis:.2f}, RY axis: {ry_axis:.2f}")

                # control forward or backword
                # map ly_axis to 0.3 to 1 as speed
                speed = float(ly_axis * 0.7) if ly_axis != 0 else 0
                # convert spped to to fixed 2 decimal places
                if speed < 0: speed-=0.3
                if speed > 0: speed+=0.3

                speed = "{:.2f}".format(speed)
                cmd = f"RUN {speed}\n"
                # print(cmd)
                ser.write(cmd.encode('utf-8'))

                # control head motor
                # map rx_axis to -90 to 90 angle
                angle = int(rx_axis * 60)
                ser.write(f"SERVO {angle}\n".encode('utf-8'))

            # Check if the program should quit
            if event.type == pygame.QUIT:
                running = False

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # Uninitialize all joysticks
    pygame.joystick.quit()
    # Quit Pygame
    pygame.quit()