import csv
import numpy as np

path = '/Users/yuzu/Documents/GitHub/MotionBlending-CA/Data/data/reaching/pos.csv'
header = ['time']
exportPath = '/Users/yuzu/Documents/GitHub/MotionBlending-CA/Data/data/reaching/sample1/time.csv'

with open(path) as file:
    reader = csv.reader(file)
    data = [row for row in reader][1:]
    data = [[float(v) for v in row] for row in data]
    data = np.array(data)

data = data[:, -1:]

with open(exportPath, 'w', newline='') as exportFile:
    writer = csv.writer(exportFile)
    writer.writerow(header)
    writer.writerows(data)