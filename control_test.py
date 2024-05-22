import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
import pygame
import time

# Pygame 초기화
pygame.init()
screen = pygame.display.set_mode((400, 300))

def connect_drone():
    # SITL 연결 문자열 변경
    connection_string = 'tcp:127.0.0.1:5760'
    print('Connecting to drone on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

def arm_and_takeoff(aTargetAltitude):
    """
    드론을 시동하고 목표 고도까지 이륙
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

vehicle = connect_drone()
arm_and_takeoff(10)

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    print("Pitch forward")
                    vehicle.channels.overrides['2'] = 2000
                elif event.key == pygame.K_DOWN:
                    print("Pitch back")
                    vehicle.channels.overrides['2'] = 1400
                elif event.key == pygame.K_LEFT:
                    print("Roll left")
                    vehicle.channels.overrides['1'] = 1400
                elif event.key == pygame.K_RIGHT:
                    print("Roll right")
                    vehicle.channels.overrides['1'] = 2000
            elif event.type == pygame.KEYUP:
                vehicle.channels.overrides = {}
except KeyboardInterrupt:
    print("Stopping vehicle")
    vehicle.channels.overrides = {}
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()

pygame.quit()
