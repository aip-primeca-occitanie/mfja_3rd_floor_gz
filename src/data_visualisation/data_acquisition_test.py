"""import nidaqmx
from nidaqmx.constants import AcquisitionType

def lire_analogique(canal="cDAQ2Mod1/ai0"):
    Lit une valeur analogique simple sur le canal spécifié.
    canal: nom du canal NI, ex: "cDAQ2Mod1/ai0"
    with nidaqmx.Task() as task:
        # Ajouter un canal analogique d'entrée
        task.ai_channels.add_ai_voltage_chan(canal)
        
        # Lire une seule valeur (blocking)
        valeur = task.read()
        print(f"Valeur lue sur {canal} : {valeur:.4f} V")

if __name__ == "__main__":
    lire_analogique()
"""

import nidaqmx
from nidaqmx.constants import AcquisitionType, READ_ALL_AVAILABLE

with nidaqmx.Task() as task:
    task.ai_channels.add_ai_voltage_chan("cDAQ2Mod1/ai1")
    task.timing.cfg_samp_clk_timing(1000.0, sample_mode=AcquisitionType.FINITE, samps_per_chan=50)
    data = task.read(READ_ALL_AVAILABLE)
    print("Acquired data: [" + ", ".join(f"{value:f}" for value in data) + "]")