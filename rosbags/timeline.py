'''
event a : /Fz_raw
event b : /Fz
event c : /cartesian_target
event d : /joint_path_command
event e : envoi ethernet (2->1)
event f : réception ethernet (1->2)
event g : /joint_states
event h : /cartesian_state
'''

import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path
from rosbags.highlevel import AnyReader
from scapy.all import rdpcap, IP, Raw

# CONFIGURATION
# ==========================

BAG_PATH  = "timing_record"
PCAP_PATH = "timing_record.pcap"
OUT_PATH  = "timeline_events.txt"

IP_1 = "172.31.0.1"
IP_2 = "172.31.0.2"

SIZE_F1 = 4 # début de l'envoi
SIZE_F2 = 132 # fin de l'envoi (le paquet intermédiaire de taille 12 est ignoré)

TOPICS = {
    "/Fz_raw" :"a",
    "/Fz" : "b",
    "/cartesian_target" : "c",
    "/joint_path_command" : "d",
    "/joint_states" : "g",
    "/cartesian_state" : "h",}

POSITION_TOPIC = "/cartesian_state"   # sert aussi de courbe de fond pour le plot


# LECTURE DU ROSBAG
# ==========================

events = [] # liste de tuples (temps, label)

t_z = []
z_values = []

with AnyReader([Path(BAG_PATH)]) as reader:

    for connection, timestamp, rawdata in reader.messages():

        topic = connection.topic

        if topic not in TOPICS:
            continue

        msg = reader.deserialize(rawdata, connection.msgtype)
        t = timestamp * 1e-9  # ns -> s

        label = TOPICS[topic]
        events.append((t, label))

        if topic == POSITION_TOPIC:
            t_z.append(t)
            z_values.append(msg.pose.position.z)

t_z = np.array(t_z)
z_values = np.array(z_values) * 100  # cm

if len(t_z) == 0:
    raise RuntimeError(f"{POSITION_TOPIC} not found")

# LECTURE DU PCAP
# ==========================

packets = rdpcap(PCAP_PATH)

pkts_2_to_1 = []   # (temps, taille_payload) pour les paquets 2 -> 1
pkts_1_to_2 = []   # (temps, taille_payload) pour les paquets 1 -> 2

for pkt in packets:
    if IP not in pkt:
        continue
    src = pkt[IP].src
    dst = pkt[IP].dst
    t = float(pkt.time)
    size = len(pkt[Raw].load) if Raw in pkt else 0

    if src == IP_2 and dst == IP_1:
        pkts_2_to_1.append((t, size))
    elif src == IP_1 and dst == IP_2:
        pkts_1_to_2.append((t, size))

pkts_2_to_1.sort(key=lambda x: x[0])
pkts_1_to_2.sort(key=lambda x: x[0])

if len(pkts_2_to_1) == 0:
    raise RuntimeError(f"sender not found")

# TEMPS 0 : PREMIER PAQUET NON NUL 2 -> 1
t0 = None
for t, size in pkts_2_to_1:
    if size > 0:
        t0 = t
        break

if t0 is None:
    raise RuntimeError("Aucun paquet non nul 2 -> 1 trouvé, impossible de définir t0")

# EVENEMENT e : ENVOI ETHERNET 2 -> 1 (paquets non nuls uniquement)
for t, size in pkts_2_to_1:
    if size > 0:
        events.append((t, "e"))

# EVENEMENTS f1 / f2 : ENVOI ETHERNET 1 -> 2 (regroupement 4 -> 12 -> 132)
for t, size in pkts_1_to_2:
    if size == SIZE_F1:
        events.append((t, "f1"))
    elif size == SIZE_F2:
        events.append((t, "f2"))
# les paquets de taille 12 (étape intermédiaire de l'envoi) ne sont pas des événements

# RECALAGE DU TEMPS 0 ET TRI CHRONOLOGIQUE
events = [(t - t0, label) for t, label in events]
events = [(t, label) for t, label in events if 0 <= t <= 12]
events.sort(key=lambda x: x[0])

t_z = t_z - t0
mask_z = (t_z >= 0) & (t_z <= 12)
t_z = t_z[mask_z]
z_values = z_values[mask_z]

# EXPORT DE LA LISTE D'EVENEMENTS
with open(OUT_PATH, "w") as f:
    for t, label in events:
        f.write(f"{t:.6f}, {label}\n")

print(f"{len(events)} événements exportés dans {OUT_PATH}")


# ANALYSE STATISTIQUE (FREQUENCE PAR TYPE D'EVENEMENT)
# ==========================

labels = sorted(set(label for _, label in events))

print("\n--- Analyse statistique des fréquences par événement ---")

for label in labels:

    t_event = np.array(sorted(t for t, l in events if l == label))

    if len(t_event) < 2:
        print(f"event {label} : pas assez d'échantillons pour calculer une fréquence")
        continue

    dt = np.diff(t_event)
    freq = 1.0 / dt

    freq_mean = np.mean(freq)
    freq_std = np.std(freq, ddof=1)

    print(f"event {label} : freq_moyenne = {freq_mean:.2f} Hz, ecart_type = {freq_std:.2f} Hz")

t_f1 = np.array(sorted(t for t, l in events if l == "f1"))
t_f2 = np.array(sorted(t for t, l in events if l == "f2"))

if len(t_f1) > 0 and len(t_f1) == len(t_f2):
    durations = t_f2 - t_f1
    print(f"\nDurée moyenne d'un envoi 1->2 (f1 -> f2) : {np.mean(durations) * 1000:.3f} ms "
          f"(ecart_type = {np.std(durations, ddof=1) * 1000:.3f} ms)")


# PLOT DE LA TIMELINE
# ==========================

# style des marqueurs par type d'événement : (marqueur, couleur)
MARKERS = {
    "a": ("*", "tab:blue"),
    "b": ("*", "tab:green"),
    "c": ("*", "tab:orange"),
    "d": ("*", "tab:purple"),
    "g": ("*", "tab:brown"),
    "h": ("*", "black"),
    "e": ("^", "red"),
    "f1": ("$[$", "tab:red"),
    "f2": ("$]$", "tab:red"),}

plt.figure(figsize=(14, 6))
plt.plot(t_z, z_values, color="gray", linewidth=1, label=POSITION_TOPIC, zorder=1)
for label in labels:
    if label=="e" or label=="d" :
        t_event = np.array([t for t, l in events if l == label])
        z_event = np.interp(t_event, t_z, z_values)
        marker, color = MARKERS.get(label, ("o", "black"))
        plt.scatter(t_event, z_event, marker=marker, color=color,
                    label=f"event {label}", zorder=2, s=80)
plt.xlabel("Temps [s]")
plt.ylabel("z [cm]")
plt.title("Timeline des événements sur /cartesian_state (z)")
plt.legend(loc="upper right", fontsize=8)
plt.grid(True)
plt.tight_layout()
plt.show()