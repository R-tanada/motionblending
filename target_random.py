import random
import time

target_right = [1, 2, 3, 4]
target_left = [1, 2, 3, 4]

target_index_right = []
target_index_left = []

while True:
    target_left = random.sample(range(1, 5), k = 2)
    target_right = random.sample(range(1, 5), k = 2)

    if (target_right[0] != target_left[0]) and (target_right[1] != target_left[1]):
        break

    time.sleep(0.5)

target = target_right[target_index_right[0]]