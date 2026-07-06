import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path
from scipy import stats
from rosbags.highlevel import AnyReader


# ==========================
# CONFIGURATION
# ==========================

BAG_PATH = "rosbag2_2026_06_09-10_42_37/"

POSITION_TOPIC = "/cartesian_state"
FORCE_TOPIC = "/Fz"
POSITION_TOL = 0.05

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

t_z = np.array(t_z)
z_values = np.array(z_values)-z_values[0]
z_values = z_values*100

t_f = np.array(t_f)
fz_values = np.array(fz_values)

if len(t_z) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {POSITION_TOPIC}")

if len(t_f) == 0:
    raise RuntimeError(f"Aucune donnée trouvée sur {FORCE_TOPIC}")

t0 = t_z[0]
t_z -= t0
t_f -= t0


#Platine de test

TXT_FORCE_FILE = "force_reference.txt"

ref_time = []
ref_fz = []

with open(TXT_FORCE_FILE, "r") as f:

    for line in f:
        line = line.strip()
        if (len(line) == 0 or line.startswith("Time")):
            continue
        parts = line.replace(",", ".").split()
        if len(parts) < 4:
            continue
        ref_time.append(float(parts[0]))
        ref_fz.append(float(parts[3]))

ref_time = np.array(ref_time)
ref_fz = np.array(ref_fz)

# Détection automatique des instants utiles (ressort en compression)

eps_z = 0.01 * np.max(np.abs(z_values))
idx_z_start = np.argmax(np.abs(z_values) > eps_z)
idx_z_end = np.max(np.where(z_values > eps_z))

t_start_ros = t_z[idx_z_start]
t_end_ros = t_z[idx_z_end]
duration = t_end_ros - t_start_ros

t_z_sync = t_z - t_start_ros
t_f_sync = t_f - t_start_ros

eps_n = 0.01 * np.max(np.abs(ref_fz))
idx_n_start = np.argmax(np.abs(ref_fz) > eps_n)
t_start_n = ref_time[idx_n_start]

t_n_sync = ref_time - t_start_n 
t_end = duration 

time_common = np.linspace(0, t_end, 5000)
z_common     = np.interp(time_common, t_z_sync, z_values)
volt_common  = np.interp(time_common, t_f_sync, fz_values)
newton_common = np.interp(time_common, t_n_sync, ref_fz)

print("données de la platine prête")

plt.figure(figsize=(12,5))
plt.plot(time_common, z_common)
plt.xlabel("Temps [s]")
plt.ylabel("z [cm]")
plt.title("Mesure position (cm)")
plt.grid(True)

plt.figure(figsize=(12,5))
plt.plot(time_common, volt_common)
plt.xlabel("Temps [s]")
plt.ylabel("Fz [V]")
plt.title("Mesure capteur (Volts)")
plt.grid(True)

plt.figure(figsize=(12,5))
plt.plot(time_common, newton_common)
plt.xlabel("Temps [s]")
plt.ylabel("Fz [N]")
plt.title("Mesure référence (Newton)")
plt.grid(True)

#détection des plateaux
plateau_indices = []
start = 0
while start < len(z_common):
    z_ref = z_common[start]
    stop = start
    while (stop < len(z_common) and abs(z_common[stop] - z_ref) < POSITION_TOL):
        stop += 1
    if stop - start > 20:
        plateau_indices.append((start, stop))
    start = stop

print(f"{len(plateau_indices)} plateaux détectés")

#moyenne des forces par plateau
z_plateau = []
volt_plateau = []
newton_plateau = []
volt_std = []

for i0, i1 in plateau_indices:
    z_plateau.append(np.mean(z_common[i0:i1]))
    volt_plateau.append(np.mean(volt_common[i0:i1]))
    newton_plateau.append(np.mean(newton_common[i0:i1]))
    volt_std.append(np.std(volt_common[i0:i1]))

volt_std = np.array(volt_std)
z_plateau = np.array(z_plateau)
volt_plateau = np.array(volt_plateau)
newton_plateau = np.array(newton_plateau)

print(f"{len(z_plateau)} points moyens conservés")

# trouver le coefficient multiplicatif V->N par moindres carrés :
# N = k * V

slope_force, intercept_force, _, _, _ = stats.linregress(volt_plateau, newton_plateau)
k_force = slope_force
print(f"k_force = {k_force:.6f} N/V")
newton_est = k_force * volt_plateau + intercept_force

#statistiques de qualité

residus = newton_plateau - newton_est
rmse = np.sqrt(np.mean(residus**2))
r2 = 1 - (np.sum(residus**2)/np.sum((newton_plateau - np.mean(newton_plateau))**2))

print()
print(f"RMSE = {rmse:.6f} N")
print(f"R² = {r2:.6f}")

# La référence N est supposée peu bruitée.

print(f"Bruit capteur (V) : std moyenne = {np.mean(volt_std):.6f} V, max = {np.max(volt_std):.6f} V")
print(f"Bruit capteur équivalent en N : {np.mean(volt_std) * k_force:.6f} N")

#raideur du ressort tant qu'à faire
# F = k z

z_m = z_plateau / 100.0
slope_ressort, intercept_ressort, _, _, _ = stats.linregress(z_m, newton_plateau)
k_ressort = slope_ressort
print(f"k_ressort = {k_ressort:.3f} N/m (intercept = {intercept_ressort:.4f} N)")

slope_fz_volt, intercept_fz_volt, _, _, _ = stats.linregress(z_m, volt_plateau)
print(f"régression F(z) en V : pente = {slope_fz_volt:.6f} V/m (intercept = {intercept_fz_volt:.6f} V)")

k_force = slope_ressort / slope_fz_volt
print(f"k = {k_force:.6f} N/V")
print()

force_ressort = k_ressort * z_m + intercept_ressort
volt_fit_z = slope_fz_volt * z_m + intercept_fz_volt

#visuels

plt.figure(figsize=(8,6))
plt.plot(z_common, volt_common, alpha=0.5, label="données")
zfit = np.linspace(z_plateau.min(), z_plateau.max(), 200) / 100.0
plt.plot(zfit * 100, slope_fz_volt * zfit + intercept_fz_volt, label=f"régression : {slope_fz_volt:.4f} V/m")
plt.xlabel("z [cm]")
plt.ylabel("Fz [V]")
plt.title("Force capteur en fonction de z")
plt.legend()
plt.grid(True)

plt.figure(figsize=(8,6))
plt.plot(z_common, newton_common, alpha=0.5, label="données")
plt.plot(zfit * 100, slope_ressort * zfit + intercept_ressort, label=f"régression : {slope_ressort:.4f} N/m")
plt.xlabel("z [cm]")
plt.ylabel("Fz [N]")
plt.title("Force référence en fonction de z")
plt.legend()
plt.grid(True)

plt.figure(figsize=(10,6))
plt.plot(z_plateau, newton_plateau, label="Mesure référence (N)")
plt.plot(z_plateau, newton_est, label="k * V")
plt.plot(z_plateau, force_ressort, label="k_ressort * z")
plt.xlabel("z [cm]")
plt.ylabel("Force [N]")
plt.title("Comparaison : mesure N / conversion kV / modèle ressort")
plt.grid(True)
plt.legend()

plt.figure(figsize=(8,6))
plt.scatter(volt_plateau, newton_plateau, s=20)
vfit = np.linspace(volt_plateau.min(), volt_plateau.max(), 200)
plt.plot(vfit, k_force * vfit + intercept_force, label=f"régression : k = {k_force:.4f} N/V")
plt.xlabel("Force mesurée [V]")
plt.ylabel("Force référence [N]")
plt.title(f"Calibration capteur : k = {k_force:.4f} N/V")
plt.legend()
plt.grid(True)

plt.show()