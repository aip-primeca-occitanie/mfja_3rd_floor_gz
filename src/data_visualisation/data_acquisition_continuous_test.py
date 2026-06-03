import nidaqmx
from nidaqmx.constants import AcquisitionType
import time

with nidaqmx.Task() as task:
    # Remplace par le bon canal
    task.ai_channels.add_ai_voltage_chan("cDAQ2Mod1/ai6")

    # Configuration du timing pour acquisition continue à 1000 Hz
    task.timing.cfg_samp_clk_timing(
        rate=1000.0,
        sample_mode=AcquisitionType.CONTINUOUS,
        samps_per_chan=1000
    )

    # Démarrage de l'acquisition
    task.start()

    print("Acquisition continue. Ctrl+C pour arrêter.\n")

    try:
        while True:
            # Lire un bloc de 1000 échantillons (environ 1 seconde)
            data = task.read(number_of_samples_per_channel=1000)
            moyenne = sum(data) / len(data)
            print(f"Moyenne tension mesurée : {moyenne:.3f} V", end='\r')
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nAcquisition arrêtée.")
        task.stop()