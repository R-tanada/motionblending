import csv
import numpy as np
from matplotlib import pyplot as plt

class Dataload:
    def __init__(self, path) -> None:
        with open(path) as file:
            reader = csv.reader(file)
            data = [row for row in reader][1:]
            data = [[float(v) for v in row] for row in data]
            data = np.array(data)
            self.data = data

time = Dataload('resource/sakurai/time_data/2_right.csv')
# user = Dataload('resource/nanri/user_data/2_left20231212_094644.csv')
auto = Dataload('resource/sakurai/auto_data/2_right.csv')
# robot = Dataload('resource/nanri/robot_data/2_left20231212_094644.csv')

x = np.linspace(0, time.data[-1], len(auto.data))

plt.plot(x, auto.data)
plt.show()