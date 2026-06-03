from csv import reader
import numpy
import matplotlib.pyplot as plt

values = list()
with open("./data_log.csv", "r") as f:
    r = reader(f, delimiter=',')
    i=0
    for line in r:
        if i > 0:
            value = numpy.array(list(map(float, line)))
            values.append(value)
        i+=1

data = numpy.zeros((3, len(values)))
for i, value in enumerate(values):
    data[:,i] = value

time = data[0]
fz = data[1]
mz = data[2]

plt.plot(time, fz, label='Fz')
plt.plot(time, mz,label='Mz')
plt.xlabel('Temps (s)')
plt.ylabel('Volts (V)')
plt.legend()
plt.grid(True)
plt.title('signaux Fz et Mz mesurés')
plt.show()

