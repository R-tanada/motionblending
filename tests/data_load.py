import csv
import numpy as np

class DataLoadManager:
    def __init__(self, path) -> None:
        self.data = 0
        self.load(path)

    def load(self, path):
        data_new = []
        with open(path) as file:
            reader = csv.reader(file)
            data = [row for row in reader][0:]
            data = [[v for v in row] for row in data]
            data = np.array(data)
            for val in data:
                elem = val[0].split()
                vol_list = []
                for vol in elem:
                    vol_list.append(float(vol) * 255)
                data_new.append(vol_list)
            
            data = np.array(data_new)
            print(data[:, 2].tolist())

manager = DataLoadManager('tests/data/jet.csv')
