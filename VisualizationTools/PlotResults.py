import matplotlib.pyplot as plt
import matplotlib.patches as patches
import itertools
import ast 

fig, ax = plt.subplots()
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_aspect('equal')

fieldnames = ['x', 'y']
TreePath = '../CodeSample/Tree.csv'
TreeXY= {}

with open(TreePath, 'r+') as csvfile:
    for field in fieldnames:
        TreeXY[field] = []
        
    for row in itertools.islice(csvfile, 1, None):
            values = list(row.strip('\n').split(","))
            for i, value in enumerate(values):
                if (not value):
                    continue 
                if (value != value ):
                    value = '0'
                nValue = ast.literal_eval(value)
                TreeXY.get(fieldnames[i]).append(nValue)

ax.plot(TreeXY['x'], TreeXY['y'], 'o', markeredgecolor="None", color='#43d5ef', ms = 2.7)

TrajPath = '../CodeSample/Trajectory.csv'
TrajXY = {}

with open(TrajPath, 'r+') as csvfile:
    for field in fieldnames:
        TrajXY[field] = []
        
    for row in itertools.islice(csvfile, 1, None):
            values = list(row.strip('\n').split(","))
            for i, value in enumerate(values):
                if (not value):
                    continue 
                if (value != value):
                    value = '0'
                nValue = ast.literal_eval(value)
                TrajXY.get(fieldnames[i]).append(nValue)
   
ax.plot(TrajXY['x'], TrajXY['y'], color='#43ef5f', lw = 3)#'o', markeredgecolor="None", 
ax.plot(TrajXY['x'][-1], TrajXY['y'][-1], 'xk', ms = 5)
                
fieldnames = ['LBX', 'LBY', 'width', 'length']
ObsPath = '../CodeSample/Obstacles.csv'
Obstacles = {}

with open(ObsPath, 'r+') as csvfile:
    for field in fieldnames:
        Obstacles[field] = []
        
    for row in itertools.islice(csvfile, 0, None):
            values = list(row.strip('\n').split(","))
            for i, value in enumerate(values):
                if (not value):
                    continue 
                if (value != value):
                    value = '0'
                nValue = ast.literal_eval(value)
                Obstacles.get(fieldnames[i]).append(nValue)
                

for i in range(len(Obstacles['LBX'])):
    ax.add_patch(patches.Rectangle((Obstacles['LBX'][i], Obstacles['LBY'][i]),
                               Obstacles['width'][i],Obstacles['length'][i],
                               facecolor = '#ff3700', edgecolor = "None"))


goal = plt.Circle((75,75), 1, color='#00ff90')
ax.add_artist(goal)

plt.show()