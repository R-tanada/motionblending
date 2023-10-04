def trapezium(x, p1 = 0.3, p2 = 0.7, w = 0):
    if x < p1:
        y = (1/p1) * x
    elif p1 < x and x < p2:
        y = 1
    elif x < p2:
        y = ((1 - w)/(p2 - 1)) * x
    return y

def liner(x, w = 0.8):
    y = w * x
    return y