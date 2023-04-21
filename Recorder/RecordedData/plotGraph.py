from matplotlib import pyplot as plt
import numpy as np
import csv
import glob

class InitGraph:
    def __init__(self) -> None:
        pass

    def InputCSV(self, datapath):
        with open(datapath) as f:
            reader = csv.reader(f)
            motion_data = [row for row in reader][1:]
            motion_data = [[float(v) for v in row] for row in motion_data]
            motion_data = np.array(motion_data)

        return motion_data

if __name__ == '__main__':
    dataPath = {'motionDef': 'motionDef/___Transform_Participant_1_20230413_1923.csv',
                'motion_noise': 'data/frec0.1_scale1_100/motion_noise/data.csv',
                'motion_com': 'data/frec0.1_scale1_200/motion_com/___Transform_Participant_1_20230419_1342.csv',
                'noise': 'data/frec0.1_scale1_100/noise/data.csv'
                }
    
    initGraph = InitGraph()
    motionDefine = initGraph.InputCSV(dataPath['motionDef'])
    motionNoise = initGraph.InputCSV(dataPath['motion_noise'])
    motionCom = initGraph.InputCSV(dataPath['motion_com'])
    noise = initGraph.InputCSV(dataPath['noise'])
    motionDef = motionDefine[:, 1:4]
    time = motionDefine[:, 0]
    motionNoise = motionNoise[:, 1:4]
    motionCom = motionCom[:, 1:4]

    length = min(len(motionNoise), len(motionCom))
    print(length)

    motionDef = motionDef[0:length]
    noise = noise[0:length]
    motionNoise = motionNoise[0:length]
    motionCom = motionCom[0:length]
    time = time[0:length]

    motionDiff = (motionNoise - motionDef/2 - noise) * 2
    motionCom = (motionCom - motionDef/2) * 2

    fig = plt.figure()

    fig_x = fig.add_subplot(2, 2, 1)
    fig_y = fig.add_subplot(2, 2, 2)
    fig_z = fig.add_subplot(2, 2, 3)

    fig_x.plot(time, noise[:, 0], label='noise')
    fig_x.plot(time, motionCom[:, 0], label='without')
    fig_x.plot(time, motionDiff[:, 0], label='with')
    
    fig_y.plot(time, noise[:, 1], label='noise')
    fig_y.plot(time, motionCom[:, 1], label='without')
    fig_y.plot(time, motionDiff[:, 1], label='with')

    fig_z.plot(time, noise[:, 2], label='noise')
    fig_z.plot(time, motionCom[:, 2], label='without')
    fig_z.plot(time, motionDiff[:, 2], label='with')

    fig_x.set_xlabel('time[s]')
    fig_y.set_xlabel('time[s]')
    fig_z.set_xlabel('time[s]')

    fig_x.set_title('axis_X',loc='right')
    fig_y.set_title('axis_Y',loc='right')
    fig_z.set_title('axis_Z',loc='right')

    fig_x.legend(loc = 'upper right') 
    fig_y.legend(loc = 'upper right') 
    fig_z.legend(loc = 'upper right') 

    plt.show()

