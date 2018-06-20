# RRT-Path-Planning
Sample of a code that uses RRT to find a path in a specified domain (c++) along with a script that allows to visualize results (python)

## User guide
In order to use the project directly from command line, one must follow the following steps: 

* Go to the build folder: 
```$ cd RRT-Path-Planning/build```

* Compile the project: 
```$ cmake .. && make -j4```

* Make visualization tools executable:
```$ cd ..```
```$ chmod +x VisualizationTools/PlotResults.py```

* Search a trajectory by running **PathPlanning**: 
```$ ./build/PathPlanning/path_planning``` (overwrites the Tree.csv, Trajectory.csv and Obstacles.csv files)

* Plot Trajectory: 
```$ VisualizationTools/PlotResults.py```


#### Note: 
All files must be launched from the main directory *RRT-Path-Planning* for the program to work.
