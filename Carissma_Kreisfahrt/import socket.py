import socket
import struct
import threading
import numpy as np
import time as tm
import pickle

import vehicle_dynamics
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.StaticParameters import StaticParameters
from vehicle_dynamics.VehicleDynamics import VehicleDynamics

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
        
        initial_state = StateVector(x=15, y=30, z=0, yaw=(3.14/180)*-35) # x=10 y=13 z=0 => Standard map x=-190 y=135 => Carissma
                                                                            #Carla benutzt grad und fahrdynamik rad

        frequency = 1000
        car_parameters_path = "bmw_m8.yaml"

        vehicle_dynamics = VehicleDynamics(initial_state = initial_state, 
                                           initial_gear = 1, 
                                           frequency = frequency, 
                                           car_parameters_path = car_parameters_path)

        for i in range(10000):   # länge simulation 1000Hz
            start_time = tm.time()

            current_state = vehicle_dynamics.tick(0.5, 0, 0.5)

            print("x:", current_state.x_a._x,                   #currentstate.x_a ist der aktuelle State Vektor
                    "y:", current_state.x_a._y,
                    "vx:", current_state.x_a.vx,
                    "yaw:", current_state.x_a.yaw,
                    "pitch:", current_state.x_a.pitch)

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
            if elapsed_time < 1:
                tm.sleep((1 - elapsed_time) / 1000)
            else:
                print("Diese Berechnung war länger als 1ms")

    except KeyboardInterrupt:
        print("Client wird beendet.")
    finally:
        client_socket.close()
        print("Client wurde beendet.")

if __name__ == "__main__":
    udp_client()
