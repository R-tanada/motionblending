import random

# 0　～ 9 までで5個
ran = random.sample(range(1, 5), k=4)
print(ran)
# print(random.sample(range(4), k=4))
# [6, 4, 3, 7, 5]