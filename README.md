# Staubli TX2‑60L — Impedance / Force Control (ROS2)

Force/impedance control stack for a **Staubli TX2‑60L** robot (room 315, MFJA) under **ROS2**, developed in the context of an internship on impedance control for **industrial drilling of aerospace parts**.

**Languages:** [English](#english) | [Français](#français)

---

## English

### 1. Overview

This repository provides:
- **`real_commander`** — the main package to drive the **real** robot: Cartesian/joint control, force‑controlled (PID) approach and contact, sinusoidal trajectory benchmarks, and real‑time visualization tools.
- **`simulation_controller`** — a simulation‑only counterpart used to validate control code (including an in‑progress drilling impedance controller) before deploying it on the real robot.
- **`rosbags`** — recording/analysis scripts (sensor calibration, latency, communication timeline).

It is built on top of the unofficial ROS2 driver for Staubli robots:
**[IvoD1998/Staubli_ROS2](https://github.com/IvoD1998/Staubli_ROS2)**.


### 2. Requirements & installation

**ROS2** workspace built with the driver above and this repo's packages sourced (`colcon build`, then `source install/setup.bash`).

**Python virtual environment** — packages to install with `pip` (test-only scripts excluded):

```bash
python3 -m venv venv
source venv/bin/activate
pip install numpy matplotlib scipy nidaqmx
```

`rclpy`, `std_msgs`, `geometry_msgs` come from the **ROS2 installation** itself (not `pip`) — make sure the venv can see the sourced ROS2 environment, or run these scripts with `--system-site-packages`.

> **Known issue:** installing the `nidaqmx` driver (NI‑DAQmx) requires checking driver/kernel compatibility. Procedure: *[TODO — to be documented]*.

### 3. Hardware startup procedure

1. Power on the robot's electrical cabinet.
2. On the teach pendant, start the VAL3 driver application (*ros_server*).
3. On the control PC, set up a network interface with IP `172.31.0.2`, subnet mask `255.255.240.0`.
4. Launch the desired `real_commander` launch file (see §4 below).
5. Start the robot‑side streaming driver:
   ```bash
   ros2 launch staubli_val3_driver robot_interface_streaming.launch.py robot_ip:=172.31.0.1
   ```

### 4. `real_commander` — commands (real robot)

**Force control with the real sensor** (stabilizes the robot around a target force via PID, Cartesian position command — e.g. spring compression):
```bash
ros2 launch real_commander force_controller_sensor.launch.py
```

**Force control with a simulated (virtual) spring**, no sensor needed:
```bash
ros2 launch real_commander force_controller_simu.launch.py
```

**Change the target force at runtime** (enter an integer value in Newtons):
```bash
ros2 run real_commander target_force_publisher
```

**Real‑time visualization** (Cartesian position, filtered force, Cartesian command, raw measured/simulated force) — run from the correct venv and directory:
```bash
python src/real_time_plotter.py
```

**Manual joint‑angle control:**
```bash
ros2 launch real_commander pos_command.launch.py
```
then, in another terminal:
```bash
ros2 run real_commander position_commander
```
Enter `<joint_id 1-6> <angle_rad>`, e.g. `1 3.14`.

**Sinusoidal Cartesian trajectory (real‑time):**
```bash
ros2 launch real_commander sinus_mvt.launch.py
```
Phase‑check visualization (delay between commanded and measured position — ~100–200 ms observed):
```bash
python src/real_time_phase_check.py
```

**Precomputed sinusoidal joint trajectory** (15 s, "ideal case" benchmark):
```bash
ros2 launch real_commander sinus_mvt_precalc.launch.py
```

### 5. `simulation_controller` — commands (simulation only)

```bash
ros2 launch simulation_controller force_controller_simu.launch.py
```
Same core nodes as `real_commander` (`cartesian_converter`, `cartesian_publisher`, `position_commander`, `target_force_publisher`); the simulated force here has no added noise. The `real_commander` visualizer can be reused (the filtered‑force curve won't show — that topic doesn't exist in simulation).

**Drilling impedance‑control simulation** (`drill_controller`, `drill_force_controller` — in progress):
```bash
ros2 launch simulation_controller drill_controller_simu.launch.py   # launch file coming soon
```

### 6. Recording data

**ROS2 topics (rosbag):**
```bash
ros2 bag record /Fz_raw /Fz /cartesian_target /joint_path_command /joint_states /cartesian_state
```
(add/remove topics as needed)

**Ethernet traffic (PC ↔ robot):**
```bash
sudo tcpdump -i enp0s31f6 host 172.31.0.1 -w capture.pcap
tcpdump -nn -r capture.pcap
```

### 7. Known issues

- Axis‑inversion bug (X and −Z) in the current joint→Cartesian conversion. Positions are correct; rotation (rx, rz) is **untested** and may misbehave — handle with care.
- Command frequency capped at **~80 Hz** (measured via rosbags / Ethernet capture, see `rosbags/timeline.py`), fairly regular. Important for drilling control since it directly affects the control loop `dt`.
- PID gains tuned in `simulation_controller` do **not** necessarily transfer well to `real_commander` — retune on the real robot.

---

## Français

### 1. Aperçu

Ce dépôt fournit :
- **`real_commander`** — le package principal pour piloter le **robot réel** : contrôle cartésien/articulaire, approche et contact asservis en force (PID), bancs d'essai en trajectoire sinusoïdale, outils de visualisation temps réel.
- **`simulation_controller`** — un équivalent en **simulation** pour valider le code de contrôle (dont un contrôleur d'impédance de perçage en cours de développement) avant déploiement sur le robot réel.
- **`rosbags`** — scripts d'enregistrement/analyse (étalonnage capteur, latences, timeline de communication).

Il s'appuie sur le driver ROS2 non officiel pour robots Staubli :
**[IvoD1998/Staubli_ROS2](https://github.com/IvoD1998/Staubli_ROS2)**.

### 2. Prérequis & installation

Workspace **ROS2** compilé avec le driver ci‑dessus et les packages de ce dépôt sourcés (`colcon build`, puis `source install/setup.bash`).

**Environnement virtuel Python** — modules à installer avec `pip` (scripts de test exclus) :

```bash
python3 -m venv venv
source venv/bin/activate
pip install numpy matplotlib scipy nidaqmx
```

`rclpy`, `std_msgs`, `geometry_msgs` proviennent de **l'installation ROS2** elle‑même (pas de `pip`) — s'assurer que le venv voit bien l'environnement ROS2 sourcé, ou lancer ces scripts avec `--system-site-packages`.

> **Problème connu :** l'installation du driver `nidaqmx` (NI‑DAQmx) nécessite de vérifier la compatibilité driver/kernel. Procédure : *[TODO — à documenter]*.

### 3. Procédure de mise en route matérielle

1. Allumer l'armoire électrique du robot.
2. Sur la tablette, activer l'application driver VAL3 (*ros_server*).
3. Sur le PC, créer une interface réseau avec l'IP `172.31.0.2`, masque `255.255.240.0`.
4. Lancer le launch file `real_commander` souhaité (voir §4 ci‑dessous).
5. Lancer le driver de streaming côté robot :
   ```bash
   ros2 launch staubli_val3_driver robot_interface_streaming.launch.py robot_ip:=172.31.0.1
   ```

### 4. `real_commander` — commandes (robot réel)

**Contrôle en force avec le capteur réel** (stabilise le robot autour d'une force cible par PID, commande en position cartésienne — ex. : compression d'un ressort) :
```bash
ros2 launch real_commander force_controller_sensor.launch.py
```

**Contrôle en force avec un ressort virtuel simulé**, sans capteur :
```bash
ros2 launch real_commander force_controller_simu.launch.py
```

**Modifier la force cible en cours d'exécution** (entrer une valeur entière en Newtons) :
```bash
ros2 run real_commander target_force_publisher
```

**Visualisation temps réel** (position cartésienne, force filtrée, commande cartésienne, force brute mesurée/simulée) — à lancer depuis le bon venv et le bon dossier :
```bash
python src/real_time_plotter.py
```

**Contrôle manuel de la position angulaire des joints :**
```bash
ros2 launch real_commander pos_command.launch.py
```
puis, dans un autre terminal :
```bash
ros2 run real_commander position_commander
```
Entrer `<id_joint 1-6> <angle_rad>`, ex. : `1 3.14`.

**Trajectoire sinusoïdale cartésienne (temps réel) :**
```bash
ros2 launch real_commander sinus_mvt.launch.py
```
Visualisation du déphasage (écart entre commande envoyée et position mesurée — ~100–200 ms observés) :
```bash
python src/real_time_phase_check.py
```

**Trajectoire sinusoïdale articulaire précalculée** (15 s, benchmark en "cas idéal") :
```bash
ros2 launch real_commander sinus_mvt_precalc.launch.py
```

### 5. `simulation_controller` — commandes (simulation uniquement)

```bash
ros2 launch simulation_controller force_controller_simu.launch.py
```
Mêmes nœuds principaux que `real_commander` (`cartesian_converter`, `cartesian_publisher`, `position_commander`, `target_force_publisher`) ; la force simulée ici n'a pas de bruit ajouté. Le visualiseur de `real_commander` reste utilisable (la courbe de force filtrée n'apparaîtra pas, ce topic n'existant pas en simulation).

**Simulation du contrôle en impédance pour le perçage** (`drill_controller`, `drill_force_controller` — en cours) :
```bash
ros2 launch simulation_controller drill_controller_simu.launch.py   # launch file à venir
```

### 6. Enregistrement des données

**Topics ROS2 (rosbag) :**
```bash
ros2 bag record /Fz_raw /Fz /cartesian_target /joint_path_command /joint_states /cartesian_state
```
(ajouter/retirer des topics selon le besoin)

**Trafic Ethernet (PC ↔ robot) :**
```bash
sudo tcpdump -i enp0s31f6 host 172.31.0.1 -w capture.pcap
tcpdump -nn -r capture.pcap
```

### 7. Problèmes connus

- Inversion des axes X et −Z dans la conversion joint→cartésien actuelle. Les positions sont correctes ; la rotation (rx, rz) est **non testée** et pourrait poser problème — à utiliser avec prudence.
- Fréquence de commande plafonnée à **~80 Hz** (mesurée via rosbags / capture Ethernet, voir `rosbags/timeline.py`), assez régulière. Point important pour le contrôle en perçage car cela affecte directement le `dt` de la boucle de commande.
- Les gains PID réglés dans `simulation_controller` ne se transposent **pas nécessairement** bien à `real_commander` — à re‑régler sur le robot réel.