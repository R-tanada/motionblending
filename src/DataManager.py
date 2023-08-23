import csv
import time
from datetime import datetime

import numpy as np
from matplotlib import pyplot as plt


class DataRecordManager():
    def __init__(self, header: list = None, fileName: str = '') -> None:
        self.data = []
        self.header = header
        self.fileName = fileName

    def record(self, data):
        self.data.append(data)

    def exportAsCSV(self):
        date = datetime.now().strftime("%Y%m%d_%H%M%S")
        with open('resource/' + self.fileName + date + '.csv', 'w', newline='') as exportFile:
            writer = csv.writer(exportFile)
            writer.writerow(self.header)
            writer.writerows(self.data)

class DataLoadManager:
    def __init__(self, path) -> None:
        self.data = 0
        self.load(path)

    def load(self, path):
        with open(path) as file:
            reader = csv.reader(file)
            data = [row for row in reader][1:]
            data = [[float(v) for v in row] for row in data]
            data = np.array(data)
            self.data_iter = iter(data)

    def getdata(self):
        try:
            self.data = next(self.data_iter)
        except StopIteration:
            pass

        return self.data

class DataPlotManager:
    def __init__(self, legend: list = None, xlabel: str = None, ylabel: str = None) -> None:
        self.data = []
        self.legend = legend
        self.xlabel = xlabel
        self.ylabel = ylabel

    def record(self, data):
        self.data.append(data)

    def plotGraph(self):
        data = np.array(self.data)
        for i in range(len(self.legend)):
            plt.plot(data[:, -1], data[:, i], label = self.legend[i])
        if self.xlabel:
            plt.xlabel(self.xlabel)
        if self.ylabel:
            plt.ylabel(self.ylabel)
        plt.vlines(x = 2.7656666000000003, ymax=400, ymin=-400, linestyles='dotted', colors=[0, 0, 0])
        plt.legend()
        plt.show()

if __name__ == '__main__':
    recorder = DataRecordManager(['x', 'y', 'z'], fileName='position')

    try:
        while True:
            recorder.record([np.random.rand(), np.random.rand(), np.random.rand()])
            time.sleep(0.01)
            # print('111')

    except KeyboardInterrupt:
        recorder.exportAsCSV()
