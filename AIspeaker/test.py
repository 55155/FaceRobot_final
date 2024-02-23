import  numpy as np
import matplotlib.pyplot as plt
def interp(x,y):
    assert len(x) == 2, 'len(x) == 2'
    assert len(y) == 2, 'len(y) == 2'
    slice_x = np.arange(x[0], x[1], 0.1)
    slice_y = []
    for i in range(slice_x.size):
        temp = ((slice_x[i] - x[1]) / (x[0] - x[1])) * y[0] + ((slice_x[i] - x[0]) / (x[1] - x[0])) * y[1]
        slice_y.append(temp)
    return slice_x, slice_y

def interp2(x,y):
    assert len(x) == 2, 'len(x) == 2'
    assert len(y) == 2, 'len(y) == 2'
    virtual_point_x = (x[0] + x[1]) / 2
    virtual_point_y = y[0] + (1/10) * (y[1] - y[0])
    print(virtual_point_x, virtual_point_y)
    slice_x = np.arange(x[0], x[1], 0.1)
    slice_y = []
    for i in range(slice_x.size):
        temp = ((slice_x[i] - virtual_point_x)*(slice_x[i] - x[1])) / ((x[0] - virtual_point_x)*(x[0] - x[1])) * y[0] \
               + ((slice_x[i] - x[0])*(slice_x[i] - x[1])) / ((virtual_point_x - x[0])*(virtual_point_x- x[1])) * virtual_point_y \
               + ((slice_x[i] - x[0])*(slice_x[i] - virtual_point_x)) / ((x[1] - x[0])*(x[1] - virtual_point_x)) * y[1]
        print(temp)
        slice_y.append(temp)
    return slice_x, slice_y, virtual_point_x, virtual_point_y

x = np.array([1, 2])
y = np.array([6, 4])

# slice_x, slice_y = interp(x,y)
slice_x, slice_y,virtual_point_x, virtual_point_y = interp2(x,y)

plt.scatter(slice_x,slice_y)
plt.scatter(virtual_point_x, virtual_point_y, cmap = 'r')
plt.show()
