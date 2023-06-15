import csv
import time
from matplotlib import pyplot as plt
import numpy as np

class DataRecordManager():
    def __init__(self, header: list = None, filename: str = None) -> None:
        self.data = []
        self.header = []
        self.header = np.hstack((header, 'time'))
        self.startTime = time.perf_counter()

    def record(self, data):
        self.data.append(np.hstack((data, time.perf_counter() - self.startTime)))

    def exportAsCSV(self, exportPath):
        with open(exportPath, 'w', newline='') as exportFile:
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