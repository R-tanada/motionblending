import csv
import time
from matplotlib import pyplot as plt
import numpy as np

class DataRecordManager():
    def __init__(self, header: list = None, exportPath: str = None) -> None:
        self.data = []
        self.header = header
        self.exportPath = exportPath

    def record(self, data):
        self.data.append(data)

    def exportAsCSV(self):
        with open(self.exportPath, 'w', newline='') as exportFile:
            writer = csv.writer(exportFile)
            writer.writerow(self.header)
            writer.writerows(self.data)

    def plotGraph(self):
        data = np.array(self.data)
        plt.plot(data[:, -1], data[:, 0:-1])
        plt.show()

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
            self.data_iter = iter(data[:, 0:-1])

    def getdata(self):
        try:
            self.data = next(self.data_iter)
        except StopIteration:
            pass

        return self.data

if __name__ == '__main__':
    recorder = DataRecordManager(['x', 'y', 'z'])

    try:
        while True:
            recorder.record([np.random.rand(), np.random.rand(), np.random.rand()])
            time.sleep(0.01)
            # print('111')

    except KeyboardInterrupt:
        recorder.exportAsCSV('Recorder/RecordedData/test.csv')