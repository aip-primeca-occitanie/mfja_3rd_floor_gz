import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path
from scipy import stats
from rosbags.highlevel import AnyReader


# ==========================
# CONFIGURATION
# ==========================

BAG_PATH = "rosbag2_2026_07_03-12_50_05"

POSITION_TOPIC = "/joint_states"
COMMAND_TOPIC = "/joint_path_command"
COMMAND_TOPIC_TRUE = "/joint_path_command"

# ==========================
# LECTURE DU BAG
# ==========================

t_z = []
z_values = []
t_command = []
command_values = []

with AnyReader([Path(BAG_PATH)]) as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = reader.deserialize(rawdata, connection.msgtype)

        if connection.topic == POSITION_TOPIC:
            t = timestamp * 1e-9  # ns -> s
            #z = msg.pose.position.z
            z = msg.position[4]
            t_z.append(t)
            z_values.append(z)
        
        elif connection.topic == COMMAND_TOPIC:
            t = timestamp * 1e-9
            for p in msg.points:
                if len(p.positions) > 1:
                    t_command.append(t)
                    command_values.append(p.positions[4])

t_z = np.array(t_z)
z_values = np.array(z_values)
z_values = z_values*100 #cm

t_command = np.array(t_command)
command_values = np.array(command_values)
command_values = command_values*100 #cm

if len(t_z) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {POSITION_TOPIC}")

if len(t_command) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {COMMAND_TOPIC}")

t0 = t_z[0]
t_z -= t0
t_command -= t0

plt.figure(figsize=(12,5))
plt.plot(t_z, z_values, label='Position')
plt.plot(t_command, command_values, label='Commande')
plt.xlabel("Temps [s]")
plt.ylabel("z [cm] / theta_2 [rad]")
plt.title("Mesure position vs Commade")
plt.grid(True)

"""
# calcul délai
from scipy.signal import find_peaks

peaks_cmd, _ = find_peaks(command_values, distance=5)
peaks_z, _ = find_peaks(z_values, distance=5)
peaks_cmd = peaks_cmd[1:]
peaks_z = peaks_z[1:]

if len(peaks_cmd) != len(peaks_z):
    print(len(peaks_cmd))
    print(len(peaks_z))
    raise RuntimeError(f"pas le meme nombre de pics trouvés")

n = len(peaks_cmd)

delays = []

for i in range(n):
    t_cmd = t_command[peaks_cmd[i]]
    t_z_peak = t_z[peaks_z[i]]
    delays.append(t_z_peak - t_cmd)

delays = np.array(delays)
mean_delay = np.mean(delays)
std_delay = np.std(delays)

print("\nDélais :")
print(f"Nb de paires : {n}")
print(f"Delai moyen : {mean_delay:.4f} s")
print(f"Écart-type  : {std_delay:.4f} s")
print(f"Delays bruts : {delays}")
"""
plt.show()