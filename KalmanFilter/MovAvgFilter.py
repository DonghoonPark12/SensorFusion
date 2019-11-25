
prevAvg
n = 10
xbuf = x *

def MovAvgFilter_batch(x):
    for i in range(n):
        xbuf[i] = xbuf[i+1]
    xbuf[n+1] = x
    avg = preAvg +
    return avg

def MovAvgFilter_recur(x):