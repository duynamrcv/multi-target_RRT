import numpy as np
from scipy import interpolate

import matplotlib.pyplot as plt

class Bezier:
    def __init__(self, start, mid, end):
        self.ctr = np.array([start,mid,mid,end])
    def evaluate(self, num):
        x = self.ctr[:, 0]; y = self.ctr[:, 1]
        l = len(x)
        t=np.linspace(0, 1, l-2,endpoint=True)
        t=np.append([0, 0, 0], t)
        t=np.append(t, [1, 1, 1])
        tck=[t, [x, y], 3]
        u3=np.linspace(0, 1,(max(l*2, num)),endpoint=True)
        out = interpolate.splev(u3, tck)
        res = [[out[0][i], out[1][i]] for i in range(len(out[0]))]
        # print(res)
        return res

if __name__ == "__main__":
    start = [3,1]
    mid = [1,2]
    end = [1,4]
    bezier = Bezier(start, mid, end)
    out = bezier.evaluate(20)

    x = [start[0], mid[0], end[0]]
    y = [start[1], mid[1], end[1]]
    plt.plot(x,y,'k--',label='Control polygon',marker='o',markerfacecolor='red')
    #plt.plot(x,y,'ro',label='Control points only')
    plt.plot([x[0] for x in out], [x[1] for x in out],'green',linewidth=2.0,label='B-spline curve')
    plt.legend(loc='best')
    plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
    plt.title('Cubic B-spline curve evaluation')
    plt.show()