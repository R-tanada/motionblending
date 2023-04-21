from matplotlib import pyplot as plt
import numpy as np
import csv

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
    initGraph = InitGraph()
    motion = initGraph.InputCSV('Minimunjerk//100/___Transform_Participant_1_20230418_1618.csv')

    #100
    position_user = motion[350:957, 1:4]
    position_min = motion[957:1440, 1:4]
    time_user = motion[350:957, 0]
    time_min = motion[957:1440, 0]

    # 150
    # position_user = motion[350:897, 1:4]
    # position_min = motion[897:1440, 1:4]
    # time_user = motion[350:897, 0]
    # time_min = motion[897:1440, 0]

    #200
    # position_user = motion[250:664, 1:4]
    # position_min = motion[664:1440, 1:4]
    # time_user = motion[250:664, 0]
    # time_min = motion[664:1440, 0]

    #250
    # position_user = motion[480:677, 1:4]
    # position_min = motion[677:1440, 1:4]
    # time_user = motion[480:677, 0]
    # time_min = motion[677:1440, 0]

    fig = plt.figure()

    fig_x = fig.add_subplot(2, 2, 1)
    fig_y = fig.add_subplot(2, 2, 2)
    fig_z = fig.add_subplot(2, 2, 3)

    fig_x.plot(time_user, position_user[:, 0], label='operator')
    fig_x.plot(time_min, position_min[:, 0], label='minimun_jerk')
    
    fig_y.plot(time_user, position_user[:, 1], label='operator')
    fig_y.plot(time_min, position_min[:, 1], label='minimun_jerk')

    fig_z.plot(time_user, position_user[:, 2], label='operator')
    fig_z.plot(time_min, position_min[:, 2], label='minimun_jerk')

    fig_x.set_xlabel('time[s]')
    fig_y.set_xlabel('time[s]')
    fig_z.set_xlabel('time[s]')

    fig_x.set_title('axis_X',loc='right')
    fig_y.set_title('axis_Y',loc='right')
    fig_z.set_title('axis_Z',loc='right')

    fig_x.legend(loc = 'right') 
    fig_y.legend(loc = 'right') 
    fig_z.legend(loc = 'right') 

    plt.show()

