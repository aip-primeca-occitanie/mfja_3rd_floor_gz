import nidaqmx
from nidaqmx.constants import AcquisitionType
import csv
import time

# --- Paramètres ---
channel_fz = "cDAQ2Mod1/ai6"  # à adapter à ton canal Fz
channel_mz = "cDAQ2Mod1/ai7"  # à adapter à ton canal Mz
sample_rate = 1000.0  # échantillons/sec

task = nidaqmx.Task()
task.ai_channels.add_ai_voltage_chan(channel_fz,min_val=-10.0, max_val=10.0)
task.ai_channels.add_ai_voltage_chan(channel_mz,min_val=-10.0, max_val=10.0)
task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.CONTINUOUS) #La carte envoie sample_rate=1000 échantillons par secondes par canal dans le buffer
task.in_stream.input_buf_size = 10000 #Définit la taille du buffer interne de la carte DAQmx pour stocker les données pendant que je les lis.

t = time.time()
task.start()

try:
    while True:
        samples = task.read(number_of_samples_per_channel=100) #Nbre d'échantillon que je lis à chaque appel de read, on lit par bloc de 100 pour ne pas surcharger le CPU
        fz_values = samples[0]
        mz_values = samples[1]
        print(f"fz_values = ",fz_values)
        print(f"mz_values = ",mz_values)

        time.sleep(0.1)  # Délai léger pour ne pas bloquer tout le CPU
except KeyboardInterrupt:
    print("Acquisition stoppée.")
finally:
    task.stop()
    task.close()
