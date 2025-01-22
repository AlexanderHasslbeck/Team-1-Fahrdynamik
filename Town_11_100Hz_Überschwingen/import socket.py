import socket
import struct
import threading
import numpy as np
import time as tm
import pickle
import math
import ast
import glob
import os
import sys

import vehicle_dynamics
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.StaticParameters import StaticParameters
from vehicle_dynamics.VehicleDynamics import VehicleDynamics

#Definitionen für Regler
L = 2.827   #Radstand
Kdd = 4.0 #Faktor look ahead distance
throttle =0.0
brake=0.0
steering=0.0


#Waypoints aus Liste laden
waypoint_list = []
with open('waypoints.txt','r') as file:
    content = file.read()
    waypoint_list=ast.literal_eval(content)
waypoint_obj_list = waypoint_list #waypoint_list

#Geschwindigkeiten aus Liste laden
speed = []
with open('speed.txt','r') as file:
    content = file.read()
    speed=ast.literal_eval(content)

# Calculate Delta
def calc_steering_angle(alpha, ld):
    delta_prev = 0
    delta = math.atan2(2 * L * np.sin(alpha), ld)
    delta = np.fmax(np.fmin(delta, 1.0), -1.0)
    if math.isnan(delta):
        delta = delta_prev
    else:
        delta_prev = delta

    return delta

# Get target waypoint index
def get_target_wp_index(veh_location, waypoint_list):
    dxl, dyl = [], []
    for i in range(len(waypoint_list)):
        dx = abs(veh_location[0] - waypoint_list[i][0])
        dxl.append(dx)
        dy = abs(veh_location[1] - waypoint_list[i][1])
        dyl.append(dy)

    dist = np.hypot(dxl, dyl)
    idx = np.argmin(dist) + 4

    # take closest waypoint, else last wp
    if idx < len(waypoint_list):
        tx = waypoint_list[idx][0]
        ty = waypoint_list[idx][1]
    else:
        tx = waypoint_list[-1][0]
        ty = waypoint_list[-1][1]

    return idx, tx, ty, dist

def get_lookahead_dist(vf, idx, waypoint_list, dist):   #calculates look ahead disance
    ld = Kdd * vf
    #while ld > dist[idx] and (idx+1) < len(waypoint_list):
    #    idx += 1
    return ld

class PIDController:
    def __init__(self, P, I, D, lim_min, lim_max):
        self.KP = P
        self.KI = I
        self.KD = D
        self.lim_min = lim_min
        self.lim_max = lim_max
        self.error_i = 0
        self.error_prev = 0

    def control(self, error, delta_time):
        error_d = (error - self.error_prev) / delta_time
        self.error_i += error
        self.error_prev = error
        result = self.KP * error + self.KI * self.error_i + self.KD * error_d
        return max(self.lim_min, min(result, self.lim_max))
        
def steuerung(controller, vx,target_speed, delta_time):
    # Geschwindigkeit des Fahrzeugs ermitteln
    global throttle
    global brake
    
    # Fehler berechnen
    error = target_speed - vx

    # Wenn der Fehler positiv ist, setzen Sie den Throttle-Wert und setzen Sie Bremse auf 0
    if error >= 0:
        throttle = controller.control(error, delta_time)
        brake = 0
    # Wenn der Fehler negativ ist, setzen Sie den Brake-Wert (machen Sie den Fehler positiv) und setzen Sie Throttle auf 0
    else:
        brake = controller.control(-error, delta_time)
        throttle = 0
    
def handle_receive(client_socket):
    while True:
        try:
            response, addr = client_socket.recvfrom(1024)
            if response:
                print(f"Antwort vom Server: {response.decode()}")
        except OSError as e:
            print(f"Fehler beim Empfangen: {e}")
            break

def udp_client(server_host='localhost', server_port=12345):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    receive_thread = threading.Thread(target=handle_receive, args=(client_socket,))
    receive_thread.daemon = True
    receive_thread.start()

    try:
        # Festen Vektor definieren
        with open("example_data/Braking.pickle", "rb") as handle:
            data = pickle.load(handle)
        
        initial_state = StateVector(x=106, y=45, z=0, yaw=(3.14/180)*270) # x=10 y=13 z=0 => Standard map x=-190 y=135 => Carissma
                                                                            #Startpunkt für Regler 
                                                                            #Carla benutzt grad und fahrdynamik rad

        frequency = 100
        car_parameters_path = "bmw_m8.yaml"

        vehicle_dynamics = VehicleDynamics(initial_state = initial_state, 
                                           initial_gear = 1, 
                                           frequency = frequency, 
                                           car_parameters_path = car_parameters_path)
        
        current_state = vehicle_dynamics.tick(0, 0, 0)
        vector = np.array([ current_state.x_a._x,
                            current_state.x_a._y,
                            current_state.x_a._z,
                            current_state.x_a.yaw,
                            current_state.x_a.roll,
                            current_state.x_a.pitch],
                            dtype=np.float32)

        for i in range(2000):   # länge simulation 1000Hz
            start_time = tm.time()
            global steering        
            
            min_index, tx, ty, dist = get_target_wp_index(vector, waypoint_list)
            ld = get_lookahead_dist(current_state.x_a.vx, min_index, waypoint_list, dist)
            alpha = math.atan2(ty - current_state.x_a._y, tx - current_state.x_a._x)-(current_state.x_a.yaw)
            if math.isnan(alpha):
                alpha = alpha_prev
            else:
                alpha_prev = alpha
            e = np.sin(alpha) * ld
            controller = PIDController(P=1,I=0.05,D=1,lim_min=0,lim_max=1)
            target_speed = speed[i]
            delta_time = 0.001
            steuerung(controller,current_state.x_a.vx,target_speed,delta_time)
            steering = calc_steering_angle(alpha, ld)
            
            current_state = vehicle_dynamics.tick(throttle, brake, steering)
            
            # print(  "x:", current_state.x_a._x,                   #currentstate.x_a ist der aktuelle State Vektor
            #         "y:", current_state.x_a._y,
            #         "vx:", current_state.x_a.vx,
            #         "yaw:", current_state.x_a.yaw,)

            vector = np.array([current_state.x_a._x,
                               current_state.x_a._y,
                               current_state.x_a._z,
                               current_state.x_a.yaw,
                               current_state.x_a.roll,
                               current_state.x_a.pitch],
                               dtype=np.float32) 
            
            array_length = len(vector)
            data = struct.pack(f'!I{array_length}f', array_length, *vector)
            client_socket.sendto(data, (server_host, server_port))

            elapsed_time = (tm.time() - start_time) * 1000
            if elapsed_time < 10:
                tm.sleep((10 - elapsed_time) / 1000)
            else:
                print("Berechnung länger als 1ms => ca.", elapsed_time, "ms")

    except KeyboardInterrupt:
        print("Client wird beendet.")
    finally:
        client_socket.close()
        print("Client wurde beendet.")

if __name__ == "__main__":
    udp_client()
