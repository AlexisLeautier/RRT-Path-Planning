# RRT-Path-Planning
Sample of a code that uses RRT to find a path in a specified domain (c++) along with a script that allows to visualize results (python)

## User guide
In order to use the project directly from command line, one must follow the following steps: 

* Go to path planning folder: 
```$ cd RRT-Path-Planning/PathPlanning```

* Compile the project and create executable: 
```$ g++ main.cpp -o PathPlanning```

* Make visualization tools executable:
```$ cd ..```
```$ chmod +x VisualizationTools/PlotResults.py```

* Search a trajectory by running **PathPlanning**: 
```$ PathPlanning\PathPlanning``` (overwrites the Tree.csv, Trajectory.csv and Obstacles.csv files)

* Plot Trajectory: 
```$ VisualizationTools/PlotResults.py```
