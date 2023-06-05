import csv
import time
from matplotlib import pyplot as plt
import numpy as np

class DataRecordManager():
    def __init__(self, header: str = None) -> None:
        self.dataRecorded = {'header': header, 'data': [], 'time': []}
        self.startTime = time.perf_counter()

    def Record(self, data):
        self.dataRecorded['data'].append(data)
        self.dataRecorded['time'].append(time.perf_counter() - self.startTime)

    def ExportAsCSV(self):
        pass

    def PlotGraph(self):
        plt.plot(self.dataRecorded['time'], self.dataRecorded['data'])
        plt.show()

if __name__ == '__main__':
    recorder = DataRecordManager()

    try:
        while True:
            recorder.Record([np.random.rand(), np.random.rand()])
            time.sleep(0.01)

    except KeyboardInterrupt:
        recorder.PlotGraph()