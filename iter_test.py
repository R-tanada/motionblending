from itertools import cycle as iter_cycle

a = [{'position': [0, 0, 0], 'rotation': [0, 0, 0]}, {'position': [1, 1, 1], 'rotation': [1, 1, 1]}]

a_iter = iter_cycle(a)

while True:
    print(next(a_iter)['position'])