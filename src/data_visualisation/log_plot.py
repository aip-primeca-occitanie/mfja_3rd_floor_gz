from csv import reader
import numpy
import matplotlib.pyplot as plt

#===== Traitement des données du fichier==========
timestamps = []
messages_info = []
messages_warning = []
messages_error = []
info_count = 0
info_freq = []
time_for_freq = []


with open("./20250724_log_errors.csv", "r") as f:
    r = reader(f, delimiter=',')
    for line in r:
        if len(line)<2:
            continue
        timestamps.append(float(line[0])*1000)
        msg = line[1].strip()
        if msg.startswith("INFO") : #Les infos correspondent aux messages obtenus quand le message est bien envoyé au robot
            messages_info.append(1)
            messages_warning.append(0)
            messages_error.append(0)
        elif msg.startswith("WARNING") :
            messages_warning.append(1)
            messages_info.append(0)
            messages_error.append(0)
        elif msg.startswith("ERROR") :
            messages_error.append(1)
            messages_warning.append(0)
            messages_info.append(0)

for t, is_info in zip(timestamps, messages_info): #zip permet d'associer les éléments de deux listes deux à deux
    if is_info:
        info_count += 1
        time_for_freq.append(t)
        # éviter division par 0
        freq = info_count / (t/1000) if t != 0 else 0
        info_freq.append(freq)


#===========Création graphe=========================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax1.step(timestamps, messages_info, label="INFO", color="blue", where='post')
ax1.step(timestamps, messages_warning, label="WARNING", color="orange", where='post')
ax1.step(timestamps, messages_error, label="ERROR", color="red", where='post')
ax1.set_xlabel("Temps (ms)")
ax1.set_ylabel("messages dans le log")
ax1.set_ylim(-0.2, 1.2)
ax1.set_title("Logs au cours du temps")
ax1.legend()
ax1.grid(True)
ax1.minorticks_on()
ax1.tick_params(axis='x', which='minor', length=4, color='gray')
ax1.grid(True, which='minor', linestyle=':', linewidth=0.5)

ax2.plot(time_for_freq, info_freq, label='Fréquence cumulée (Hz)', color='blue')
ax2.set_ylabel('Fréquence [Hz]')
ax2.set_xlabel('Temps [ms]')
ax2.legend()
ax2.grid(True)


plt.tight_layout()
plt.show()

