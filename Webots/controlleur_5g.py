from controller import Robot, Emitter, Receiver, GPS, Compass
import json
import random
import numpy as np

# --- INITIALISATION ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
DRONE_ID = robot.getName() 

# NOMS MIS À JOUR POUR CORRESPONDRE À TA VIDÉO (Formation Statique)
TARGET_FORMATION = {
    "e-puck(1)": [-0.2, 0.0],     # mon robot gauche
    "e-puck(2)": [0.0, 0.2],      # mon robot du milieu (monte)
    "e-puck(3)": [0.2, 0.0]       # le robot droit
}

SAFE_DISTANCE = 0.15
MODE_5G = True
LATENCY_BASE = 0.020 if MODE_5G else 0.100
JITTER_MAX = 0.005 if MODE_5G else 0.050
PACKET_LOSS_PROB = 0.01 if MODE_5G else 0.10
MAX_SPEED = 5.0
P_GAIN = 2.0

# --- PÉRIPHÉRIQUES ---
gps = robot.getDevice("gps")
if gps: 
    gps.enable(timestep)
else:
    print(f"[{DRONE_ID}] ALERTE : Aucun capteur GPS trouvé dans l'arbre de la scène !")

compass = robot.getDevice("compass")
if compass: 
    compass.enable(timestep)
else:
    print(f"[{DRONE_ID}] ALERTE : Aucune boussole trouvée dans l'arbre de la scène !")

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
if emitter: emitter.setChannel(1)
if receiver: 
    receiver.enable(timestep)
    receiver.setChannel(1)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
if left_motor: 
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
if right_motor: 
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

other_drones_data = {}

# BOUCLE PRINCIPALE 
while robot.step(timestep) != -1:
    t = robot.getTime()
    
    if DRONE_ID not in TARGET_FORMATION:
        continue

    # Récupération GPS
    if gps: 
        position = gps.getValues()[:2]
    else: 
        position = [0.0, 0.0]

    # Récupération Boussole
    current_yaw = 0.0
    if compass:
        comp_val = compass.getValues()
        current_yaw = np.arctan2(comp_val[0], comp_val[1])

    # Communication 5G
    if t % 0.1 < timestep/1000.0:
        data_to_send = {"drone_id": DRONE_ID, "position": position, "timestamp": t}
        if emitter:
            if random.random() > PACKET_LOSS_PROB:
                emitter.send(json.dumps(data_to_send).encode('utf-8'))

    # Réception 5G (Avec simulation de latence et LOGS)
    if receiver and receiver.getQueueLength() > 0:
        while receiver.getQueueLength() > 0:
            try:
                data = json.loads(receiver.getString())
                
                packet_time = data["timestamp"]
                simulated_latency = LATENCY_BASE + random.uniform(0, JITTER_MAX)
                
                # Le paquet a-t-il voyagé assez longtemps ?
                if (t - packet_time) >= simulated_latency:
                    other_drones_data[data["drone_id"]] = data
                    
                    # LOGS CLAIRS DANS LA CONSOLE
                    latence_ms = (t - packet_time) * 1000
                    print(f"📡 [{DRONE_ID}] a reçu {data['drone_id']} | Latence: {latence_ms:.1f} ms")
                        
                    receiver.nextPacket() 
                else:
                    break # On simule le délai de transmission
            except: 
                receiver.nextPacket()

    # Coordination & Évitement
    target_position = TARGET_FORMATION[DRONE_ID]
    for drone_id, data in other_drones_data.items():
        if drone_id != DRONE_ID:
            other_pos = np.array(data["position"])
            my_pos = np.array(position)
            distance = np.linalg.norm(other_pos - my_pos)
            
            # SÉCURITÉ ANTI-CRASH
            if 0.001 < distance < SAFE_DISTANCE:
                avoidance_vector = (my_pos - other_pos) / distance
                target_position = np.array(target_position) + 0.5 * avoidance_vector

    # Déplacement et Arrêt
    x_error = target_position[0] - position[0]
    y_error = target_position[1] - position[1]
    distance_to_target = np.sqrt(x_error**2 + y_error**2)
    
    # Ils s'arrêtent s'ils sont à moins de 2 cm de leur cible
    if distance_to_target < 0.02:
        left_speed = 0.0
        right_speed = 0.0
    else:
        target_angle = np.arctan2(y_error, x_error)
        angle_error = target_angle - current_yaw
        
        while angle_error > np.pi: angle_error -= 2 * np.pi
        while angle_error < -np.pi: angle_error += 2 * np.pi
        
        speed = P_GAIN * distance_to_target
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, speed - angle_error * 3.0))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, speed + angle_error * 3.0))

    if left_motor: left_motor.setVelocity(left_speed)
    if right_motor: right_motor.setVelocity(right_speed)