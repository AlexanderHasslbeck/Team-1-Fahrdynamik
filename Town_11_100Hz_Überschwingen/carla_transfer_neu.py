
import carla
import time as tm
import socket
import struct
import threading
import numpy as np

# Globale Variable
vector = np.array([0, 0, 0, 0], dtype=np.float32)
vector_received = threading.Event()

def handle_receive(server_socket):
    global vector
    while True:
        try:
            data, addr = server_socket.recvfrom(1024)
            if data:
                array_length = struct.unpack('!I', data[:4])[0]
                array_data = struct.unpack(f'!{array_length}f', data[4:])
                vector = np.array(array_data, dtype=np.float32)
                print(f"Empfangener Vektor vom Client {addr}: {vector}")
                vector_received.set()  # Setze das Event, um anzuzeigen, dass ein neuer Vektor empfangen wurde
        except OSError as e:
            print(f"Fehler beim Empfangen: {e}")
            break

def udp_server(host='localhost', port=12345):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((host, port))
    print(f"UDP Server gestartet und hört auf {host}:{port}")

    receive_thread = threading.Thread(target=handle_receive, args=(server_socket,))
    receive_thread.daemon = True
    receive_thread.start()

    return server_socket

def spawn_vehicle_at_vector(world, blueprint_library, vector):
    # Ein Blueprint für ein Fahrzeug finden
    vehicle_bp = blueprint_library.filter('vehicle.*')[0]  # Wählen Sie das erste Fahrzeug

    # Definieren Sie den Spawnpunkt
    spawn_point = carla.Transform(
        carla.Location(x=float(vector[0]), y=float(vector[1]), z=float(vector[2])),
        carla.Rotation(yaw=(float(vector[3])/3.14)*180, roll=(float(vector[4])/3.14)*180, pitch=(float(vector[5])/3.14)*180)
    )

    # Fahrzeug spawnen
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle_id = vehicle.id
    print(f'Fahrzeug {vehicle.type_id} wurde bei {spawn_point.location} gespawnt mit ID {vehicle_id}')

    return vehicle


def teleport_vehicle_to_vector(vehicle, vector):
    # Definieren Sie den neuen Standort
    new_location = carla.Location(x=float(vector[0]), y=float(vector[1]), z=float(vector[2]))
    new_rotation = carla.Rotation(yaw=(float(vector[3])/3.14)*180, roll=(float(vector[4])/3.14)*180, pitch=(float(vector[5])/3.14)*180)

    # Fahrzeug teleportieren
    vehicle.set_transform(carla.Transform(new_location, new_rotation))
    print(f'Fahrzeug zu neuem Vektor teleportiert: {new_location}')

def main():
    global vector
    try:
        # Verbindung zum CARLA-Server herstellen
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Die Welt und die Blueprint-Bibliothek abrufen
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # Starten des UDP-Servers in einem separaten Thread
        server_socket = udp_server()

        # Hauptschleife, die auf Vektor-Empfang wartet und das Fahrzeug spawnt oder teleportiert
        current_vehicle = None
        while True:
            vector_received.wait()  # Warte, bis ein neuer Vektor empfangen wird
            if current_vehicle:
                teleport_vehicle_to_vector(current_vehicle, vector)
            else:
                current_vehicle = spawn_vehicle_at_vector(world, blueprint_library, vector)

            vector_received.clear()  # Setze das Event zurück

    except Exception as e:
        print(e)

    finally:
        if 'server_socket' in locals():
            server_socket.close()    #if current_vehicle:
                        
            current_vehicle.destroy()
            print('Letztes Fahrzeug zerstört')

if __name__ == '__main__':
    main()
