import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import imageio


# plt.style.use('seaborn-pastel')

PART = 1000
ITER = 100

f = open("/home/anton/cpp/project/dump.txt", 'r')
def animate(i):
    xs = []
    ys = []
    for _ in range(i*PART, (i+1) * PART):
        linee = f.readline()
        xs.append(float(linee.split()[0]))
        ys.append(float(linee.split()[1]))

    fig = plt.figure(figsize=(6, 6))
    plt.plot(xs, ys, 'b.' )
    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.xlabel('x', fontsize = 14)
    plt.ylabel('y', fontsize = 14)
    plt.savefig(f'./img/img_{i}.png',
                transparent = False,  
                facecolor = 'white'
               )
    plt.close()
    return plt.savefig


frames = []
for i in range(ITER-1):
    frames.append(animate(i))


# imageio.mimsave('./example.gif', # output gif
#                 frames,          # array of input frames
#                 fps = 5)         # optional: frames per second
f.close()