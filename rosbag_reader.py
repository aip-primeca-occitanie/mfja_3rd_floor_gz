#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path
from rosbags.highlevel import AnyReader


# ==========================
# CONFIGURATION
# ==========================

BAG_PATH = "/home/tiago/staubli_ws/rosbag2_2026_06_04-09_45_24/"

POSITION_TOPIC = "/cartesian_state"
FORCE_TOPIC = "/Fz"


# ==========================
# LECTURE DU BAG
# ==========================

t_z = []
z_values = []


t_f = []
fz_values = []

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


# ==========================
# CONVERSION NUMPY
# ==========================

t_z = np.array(t_z)
z_values = np.array(z_values)-z_values[0]
z_values = z_values*100

t_f = np.array(t_f)
fz_values = np.array(fz_values)

if len(t_z) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {POSITION_TOPIC}")

if len(t_f) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {FORCE_TOPIC}")

# ==========================
# TEMPS RELATIF
# ==========================

t0 = t_z[0]

t_z -= t0
t_f -= t0


# ==========================
# INTERPOLATION FORCE
# ==========================

fz_interp = np.interp(
    t_z,
    t_f,
    fz_values
)


# ==========================
# FIGURE 1 : z(t)
# ==========================

plt.figure(figsize=(10, 5))

plt.plot(t_z, z_values)

plt.xlabel("Temps [s]")
plt.ylabel("z [cm]")
plt.title("Position Z")
plt.grid(True)


# ==========================
# FIGURE 2 : Fz(t)
# ==========================

plt.figure(figsize=(10, 5))

plt.plot(t_f, fz_values)

plt.xlabel("Temps [s]")
plt.ylabel("Fz [V]")
plt.title("Force Z")
plt.grid(True)

# ==========================
# FIGURE 4 : Fz(z)
# ==========================

plt.figure(figsize=(8, 6))

plt.plot(z_values, fz_interp)

plt.xlabel("z [cm]")
plt.ylabel("Fz [V]")
plt.title("Force en fonction de la position")
plt.grid(True)


plt.show()