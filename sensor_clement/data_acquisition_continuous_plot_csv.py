import nidaqmx
from nidaqmx.constants import AcquisitionType
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
import time
from collections import deque
import numpy as np
from scipy import stats as sp_stats

print("start")
system = nidaqmx.system.System.local()
print(list(nidaqmx.system.System.local().devices))
print("end")

# --- Paramètres ---
channel_fz = "cDAQ2Mod1/ai6"  # à adapter à ton canal Fz
channel_mz = "cDAQ2Mod1/ai7"  # à adapter à ton canal Mz
sample_rate = 1000.0  # échantillons/sec
csv_filename = "acquisition.csv"
buffer_size = 1000
animation_interval_ms = 100 # Intervalle de mise à jour de l'animation en ms, correspond à 'interval' de FuncAnimation

# --- Initialisation des buffers pour le tracé ---
time_data = deque(maxlen=buffer_size)
fz_data = deque(maxlen=buffer_size)
mz_data = deque(maxlen=buffer_size)

# --- Préparation du fichier CSV ---
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Timestamp', 'Fz (N)', 'Mz (V)'])

# --- Initialisation de la tâche NI-DAQ ---
task = nidaqmx.Task()
task.ai_channels.add_ai_voltage_chan(channel_fz,min_val=-10.0, max_val=10.0)
task.ai_channels.add_ai_voltage_chan(channel_mz,min_val=-10.0, max_val=10.0)
task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.CONTINUOUS)

start_time = time.time()

# --- Fonction appelée à chaque frame de l'animation ---
def update(frame):
    # Calcule le nombre d'échantillons à lire pour cette frame
    # Cela dépend de votre sample_rate et de l'intervalle de l'animation
    # Si interval=100ms (0.1s) et sample_rate=1000Hz, lisez 100 échantillons.
    samples_to_read = int(sample_rate * (animation_interval_ms / 1000.0))
    if samples_to_read < 1 : # Assurez-vous de lire au moins 1 échantillon
        samples_to_read = 1

    try :
        samples_block = task.read(number_of_samples_per_channel=samples_to_read)
    except nidaqmx.errors.DaqError as e:
        print(f"Erreur DAQ : {e}")
        return line1, line2


    #samples = task.read(number_of_samples_per_channel=1)
    #t = time.time() - start_time
    fz_values = samples_block[0]
    mz_values = samples_block[1]

    for i in range(samples_to_read):
        t = time.time() - start_time #Attention, approche simplifiée
        fz = fz_values[i] * 20.864489
        mz = mz_values[i]

        # Stockage en mémoire
        time_data.append(t)
        fz_data.append(fz)
        mz_data.append(mz)

        # Enregistrement CSV
        csv_writer.writerow([t, fz, mz])

    # Mise à jour des courbes
    line1.set_data(time_data, fz_data)
    line2.set_data(time_data, mz_data)
    ax.relim()
    ax.autoscale_view()

    return line1, line2

# --- Création de la figure matplotlib ---
fig, ax = plt.subplots()
line1, = ax.plot([], [], label="Fz (N)")
line2, = ax.plot([], [], label="Mz (V)")
ax.set_title("Lecture en temps réel")
ax.set_xlabel("Temps (s)")
ax.set_ylabel("Tension/Force (N)")
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

# --- Analyse statistique du bruit ---
if len(fz_data) > 10:
    fz_arr = np.array(list(fz_data))
    mu, sigma = np.mean(fz_arr), np.std(fz_arr)
    R = sigma ** 2
    diffs = np.diff(fz_arr)
    Q = max(np.var(diffs) / 2.0 - R, R / 100.0)
    spike_rate = np.sum(np.abs(fz_arr - np.median(fz_arr)) > 3*sigma) / (len(fz_arr) / sample_rate)
    p_norm = sp_stats.normaltest(fz_arr)[1]
    print(f"\n=== Paramètres Kalman ===")
    print(f"μ = {mu:.4f} N")
    print(f"σ = {sigma:.4f} N")
    print(f"R = {R:.6f}  (variance mesure)")
    print(f"Q = {Q:.6f}  (bruit processus estimé)")
    print(f"Spikes = {spike_rate:.1f}")
    print(f"p_norm = {p_norm:.4f}")