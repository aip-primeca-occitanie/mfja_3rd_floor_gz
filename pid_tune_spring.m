k = 2408;

s = tf('s');
G = k/s;

pidTuner(G,'PID')