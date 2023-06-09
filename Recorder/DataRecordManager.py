import csv
import time
from matplotlib import pyplot as plt
import numpy as np

class DataRecordManager():
    def __init__(self, header: list = None) -> None:
        self.data = []
        self.header = []
        self.header = np.hstack((header, 'time'))
        self.startTime = time.perf_counter()

    def Record(self, data):
        self.data.append(np.hstack((data, time.perf_counter() - self.startTime)))

    def ExportAsCSV(self, exportPath):
        with open(exportPath, 'w') as exportFile:
            writer = csv.writer(exportFile)
            writer.writerow(self.header)
            writer.writerows(self.data)

    # def PlotGraph(self):
    #     plt.plot(self.dataRecorded['time'], self.dataRecorded['data'])
    #     plt.show()

if __name__ == '__main__':
    recorder = DataRecordManager(['x', 'y', 'z'])

    try:
        while True:
            recorder.Record([np.random.rand(), np.random.rand(), np.random.rand()])
            time.sleep(0.01)
            # print('111')

    except KeyboardInterrupt:
        recorder.ExportAsCSV('Recorder/RecordedData/test.csv')