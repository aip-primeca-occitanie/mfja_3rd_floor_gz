import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path
from scipy import stats
from rosbags.highlevel import AnyReader


# ==========================
# CONFIGURATION
# ==========================

BAG_PATH = "rosbag2_2026_06_16-12_40_33"

POSITION_TOPIC = "/cartesian_state"
FORCE_TOPIC = "/Fz"
COMMAND_TOPIC = "/joint_path_command"

# ==========================
# LECTURE DU BAG
# ==========================

t_z = []
z_values = []
t_f = []
fz_values = []
t_command = []
command_values = []

with AnyReader([Path(BAG_PATH)]) as reader:

    for connection, timestamp, rawdata in reader.messages():

        msg = reader.deserialize(rawdata, connection.msgtype)

        if connection.topic == POSITION_TOPIC:

            t = timestamp * 1e-9  # ns -> s

            # Première valeur du tableau
            z = msg.pose.position.x

            t_z.append(t)
            z_values.append(z)

        elif connection.topic == FORCE_TOPIC:

            t = timestamp * 1e-9  # ns -> s

            fz = msg.data

            t_f.append(t)
            fz_values.append(fz)
        
        elif connection.topic == COMMAND_TOPIC:

            t = timestamp * 1e-9  # ns -> s

            fz = msg.data

            t_f.append(t)
            fz_values.append(fz)

t_z = np.array(t_z)
z_values = np.array(z_values)-z_values[0]
z_values = z_values*100 #cm

t_f = np.array(t_f)
fz_values = np.array(fz_values)

t_command = np.array(t_command)
command_values = np.array(command_values)
command_values = command_values*100 #cm

if len(t_z) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {POSITION_TOPIC}")

if len(t_f) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {FORCE_TOPIC}")

if len(t_command) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {FORCE_TOPIC}")

t0 = t_z[0]
t_z -= t0
t_f -= t0
t_command -= t0

plt.figure(figsize=(12,5))
plt.plot(t_z, z_values)
plt.xlabel("Temps [s]")
plt.ylabel("z [cm]")
plt.title("Mesure position (cm)")
plt.grid(True)

plt.figure(figsize=(12,5))
plt.plot(t_f, fz_values)
plt.xlabel("Temps [s]")
plt.ylabel("Fz simulée [N]")
plt.title("Mesure force")
plt.grid(True)

plt.figure(figsize=(12,5))
plt.plot(t_command, command_values)
plt.xlabel("Temps [s]")
plt.ylabel("Commande [cm]")
plt.title("commande calculée")
plt.grid(True)

print("freq = ",len(t_command)/(t_command[-1]-t_command[0])," Hz.")