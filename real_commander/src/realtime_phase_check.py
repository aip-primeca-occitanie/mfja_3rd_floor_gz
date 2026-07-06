import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

WINDOW_S = 1.5  # durée affichée en secondes

class RealtimePlotterNode(Node):
    def __init__(self):
        super().__init__('realtime_plotter_node')

        self.lock = threading.Lock()
        self.t_z, self.z_vals = [], [] # position z (cm)
        self.t_cmd, self.cmd_vals = [], [] # commande delta z (cm)
        self.t0 = None
        self.create_subscription(PoseStamped, '/cartesian_state', self.cb_pos, 10)
        self.create_subscription(PoseStamped, '/cartesian_target', self.cb_cmd, 10)

    def _now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def trim(self, t_list, v_list, t_now): #Supprime les points plus vieux que WINDOW_S.
        cutoff = t_now - WINDOW_S
        while t_list and t_list[0] < cutoff:
            t_list.pop(0)
            v_list.pop(0)

    def cb_pos(self, msg):
        t = self._now()
        with self.lock:
            self.t_z.append(t)
            self.z_vals.append(msg.pose.position.z * 100) # cm
            self.trim(self.t_z, self.z_vals, t)

    def cb_cmd(self, msg):
        t = self._now()
        with self.lock:
            self.t_cmd.append(t)
            if len(self.z_vals)>0 :
                self.cmd_vals.append(- msg.pose.position.z * 100) # + self.z_vals[-1] ) #cm /!\ a verif
            else :
                self.cmd_vals.append(0.)
            self.trim(self.t_cmd, self.cmd_vals, t)

def main():
    rclpy.init()
    node = RealtimePlotterNode()

    # ROS tourne dans son propre thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    fig, (ax) = plt.subplots(1, 1, figsize=(10, 8))

    line_z, = ax.plot([], [], color="#1100ff", linewidth=1.0, label = "Position z (cm)")
    line_cmd, = ax.plot([], [], color="#15ff00", linewidth=1.0, label = "Commande z (cm)")

    ax.set_title("Position vs Commande")
    ax.set_xlabel("Temps (s)")
    ax.grid(True, alpha=0.3)

    def update(_frame):
        with node.lock:
            t_z = list(node.t_z); z = list(node.z_vals)
            t_cmd = list(node.t_cmd); cmd = list(node.cmd_vals)

        for line, ax, t, v in [(line_z, ax, t_z, z), (line_cmd, ax, t_cmd, cmd)]:
            if len(t) < 2:
                continue
            line.set_data(t, v)
            t_now = t[-1]
            ax.set_xlim(t_now - WINDOW_S, t_now)
            ax.relim()
            ax.autoscale_view(scalex=False)
        return line_z, line_cmd

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)

    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()