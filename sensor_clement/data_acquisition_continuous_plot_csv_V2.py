import nidaqmx
from nidaqmx.constants import AcquisitionType
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
import time
from collections import deque

system = nidaqmx.system.System.local()
print(list(nidaqmx.system.System.local().devices))

# --- Paramètres ---
channel_mz = "cDAQ1Mod1/ai6"  # à adapter à ton canal Fz
channel_fz = "cDAQ1Mod1/ai7"  # à adapter à ton canal Mz
sample_rate = 1000.0  # échantillons/sec
csv_filename = "acquisition.csv"
buffer_size = 50000
animation_interval_ms = 10 # Intervalle de mise à jour de l'animation en ms, correspond à 'interval' de FuncAnimation

# --- Initialisation des buffers pour le tracé ---
time_data = deque(maxlen=buffer_size)
fz_data = deque(maxlen=buffer_size)
mz_data = deque(maxlen=buffer_size)

# --- Préparation du fichier CSV ---
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Timestamp', 'Fz (V)', 'Mz (V)'])

# --- Initialisation de la tâche NI-DAQ ---
task = nidaqmx.Task()
task.ai_channels.add_ai_voltage_chan(channel_fz,min_val=-10.0, max_val=10.0)
task.ai_channels.add_ai_voltage_chan(channel_mz,min_val=-10.0, max_val=10.0)

# 1. Configurer une horloge d'échantillonnage
task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.CONTINUOUS)

# 2. Augmenter la taille du buffer matériel (ex: 2 secondes de données)
task.in_stream.input_buf_size = int(sample_rate * 2)

start_time = time.time()

# --- Fonction appelée à chaque frame de l'animation ---
def update(frame):
    
    # 3. Lire TOUT ce qui est disponible dans le buffer
    # Utiliser timeout=0 pour ne pas bloquer l'animation
    try:
        # read_all_available=True permet de vider le buffer actuel
        samples_block = task.read(number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE, timeout=0.1)
        
        # Gestion des données (si 1 seul échantillon, read retourne une liste simple, sinon une liste de listes)
        if isinstance(samples_block[0], float):
            fz_values, mz_values = [samples_block[0]], [samples_block[1]]
        else:
            fz_values, mz_values = samples_block[0], samples_block[1]

        for i in range(len(fz_values)):
            t = time.time() - start_time
            time_data.append(t)
            fz_data.append(fz_values[i])
            mz_data.append(mz_values[i])
            csv_writer.writerow([t, fz_values[i], mz_values[i]])

        line1.set_data(time_data, fz_data)
        line2.set_data(time_data, mz_data)
        ax.relim()
        ax.autoscale_view()
        
    except nidaqmx.errors.DaqError as e:
        # L'erreur -200284 (timeout) est normale si le buffer est vide, on l'ignore
        if e.error_code != -200284:
            print(f"Erreur DAQ : {e}")

    return line1, line2

# --- Création de la figure matplotlib ---
fig, ax = plt.subplots()
line1, = ax.plot([], [], label="Fz (V)")
line2, = ax.plot([], [], label="Mz (V)")
ax.set_title("Lecture en temps réel")
ax.set_xlabel("Temps (s)")
ax.set_ylabel("Tension (V)")
ax.legend()
ax.grid()

# --- Lancement de l'acquisition ---
task.start()
ani = animation.FuncAnimation(fig, update, animation_interval_ms)

print(f"Acquisition en cours... (fichier CSV : {csv_filename})\nFerme la fenêtre pour arrêter.")

# --- Affichage du graphique ---
plt.show()

# --- Nettoyage une fois la fenêtre fermée ---
task.stop()
task.close()
csv_file.close()
print("Acquisition terminée et fichier CSV sauvegardé.")
