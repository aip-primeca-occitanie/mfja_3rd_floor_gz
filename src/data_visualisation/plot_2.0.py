from csv import reader
import numpy
import matplotlib.pyplot as plt

#===== Traitement des données du fichier DAQmx==========
daqmx_values = list()
with open("./data_log.csv", "r") as f:
    r = reader(f, delimiter=',')
    i=0
    for line in r:
        if i > 0:
            daqmx_value = numpy.array(list(map(float, line)))
            daqmx_values.append(daqmx_value)
        i+=1

daqmx_data = numpy.zeros((3, len(daqmx_values)))
for i, daqmx_value in enumerate(daqmx_values):
    daqmx_data[:,i] = daqmx_value

time_daq = daqmx_data[0]
fz_V = daqmx_data[1]
mz_V = daqmx_data[2]

# Vu sur WITIS : pour Fz : Y = 20 x et pour Mz : Y=2x pour passer en newton
#data_N = numpy.zeros((3, len(daqmx_values)))
#for i, daqmx_value in enumerate(daqmx_values):
#   data_N[:,i] = daqmx_value

fz_N = fz_V * 20
mz_N = mz_V * 2

#================ FIN =================================

#======= Traitement des données de commande======
command_values = list()
with open("./cmd_trajectory.csv", "r") as f: #Le fichier se situe dans /devel/controller/build
    r = reader(f, delimiter=',')
    i=0
    for line in r:
        if i > 0:
            command_value = numpy.array(list(map(float, line)))
            command_values.append(command_value)
        i+=1

command_data = numpy.zeros((2, len(command_values)))
for i, command_value in enumerate(command_values):
    command_data[:,i] = command_value

time_cmd = command_data[0]
delta_z = command_data[1]
delta_z_mm = delta_z * 1000 #La donnée brute est en mètre


#===========================================================

#===========Création des graphes=========================
fig, (ax1, ax2) = plt.subplots(2,1,figsize=(10,8))

# subplot 1 : Signaux en Newtons
ax1.plot(time_daq, fz_N, label='Fz (N)', color='blue')
ax1.plot(time_daq, mz_N, label='Mz (N.m)', color='green')
ax1.set_xlabel("Temps (s)")
ax1.set_ylabel("Newton (N)")
ax1.set_title("Signaux Fz et Mz mesurés (N et N.m)")
ax1.grid(True)
ax1.legend()

# Subplot 2 : Signaux delta_z
ax2.plot(time_cmd, delta_z_mm,'o--', label='delta z commandé', color='purple')
ax2.set_xlabel("Temps (ms)")
ax2.set_ylabel("delta (mm)")
ax2.set_title("commande delta z")
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()








#plt.plot(time, fz, label='Fz')
#plt.plot(time, mz,label='Mz')
#plt.xlabel('Temps (s)')
#plt.ylabel('Volts (V)')
#plt.legend()
#plt.grid(True)
#plt.title('signaux Fz et Mz mesurés')
#plt.show()

