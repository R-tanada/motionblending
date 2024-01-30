import csv
import numpy as np

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


