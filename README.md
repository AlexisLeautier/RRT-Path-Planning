# RRT-Path-Planning
Sample of a code that uses RRT to find a path in a specified domain (c++) along with a script that allows to visualize results (python)

## User guide
In order to use the project directly from command line, one must follow the following steps: 

* `cd` into the directory: 
```$ cd RRT-Path-Planning/```

* Compile the project: 
```$ mkdir build && cd build && cmake .. && make -j4```

* [Optional] Make visualization tools executable:
```$ cd ..```
```$ chmod +x VisualizationTools/PlotResults.py```

* Search a trajectory by running **PathPlanning**: 
```$ ./build/PathPlanning/path_planning``` (overwrites the Tree.csv, Trajectory.csv and Obstacles.csv files)

* Plot Trajectory: 
```$ VisualizationTools/PlotResults.py```
or, if you skipped the optional step: 
```$ python VisualizationTools/PlotResults.py```

