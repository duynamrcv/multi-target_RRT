# Multi-target RRT for UAVs path planning

This repository presents the following article in Python:

Thu Hang Khuat, Duy-Nam Bui, Hoa TT. Nguyen, Mien L. Trinh , Minh T. Nguyen , Manh Duong Phung, "**Multi-goal Rapidly Exploring Random Tree with Safety and Dynamic Constraints for UAV Cooperative Path Planning**," *IEEE Transactions on Vehicular Technology*. 
[[**IEEE** *Xplore*](https://ieeexplore.ieee.org/document/10964594)] [[Citation](#citation)]

## Installation
```
git@github.com:duynamrcv/multi-target_RRT.git
```

## Run simulation
The current version have four different scenarios, from 1 to 4. Before run our method, please change the value of `scenario` in file. To run our method, run:
```
python3 contraint_rrt.py
python3 post_processing.py
```

We also implement some methods to conduct the comparisons, i.e. `compare_*.py`. To run those methods, run:
```
python3 compare_rrt_star.py
python3 compare_rrt_smart.py
```

The data will be saved in `*.txt` file. This file is used for the statistic, i.e. `analys_*.py`.

## Results and Comparison
### Path
<img src="result/path.jpeg" alt="Path" width="100%" >

### Comparisons
|  |  |   |
| :---:        |     :---:      | :---:        |
|  <img src="result/length.jpeg" alt="" width="100%" >   | <img src="result/smooth.jpeg" alt="" width="100%" >    |   <img src="result/time.jpeg" alt="" width="100%">


## Citation
```
@ARTICLE{10964594,
  author={Khuat, Thu Hang and Bui, Duy-Nam and Nguyen, Hoa TT. and Trinh, Mien L. and Nguyen, Minh T. and Phung, Manh Duong},
  journal={IEEE Transactions on Vehicular Technology}, 
  title={Multi-goal Rapidly Exploring Random Tree with Safety and Dynamic Constraints for UAV Cooperative Path Planning}, 
  year={2025},
  volume={},
  number={},
  pages={1-12},
  keywords={Autonomous aerial vehicles;Path planning;Vehicle dynamics;Turning;Heuristic algorithms;Peer-to-peer computing;Training;Particle swarm optimization;Mathematical programming;Robots;Cooperative path planning;rapidly exploring random tree;unmanned aerial vehicle},
  doi={10.1109/TVT.2025.3560658}}
```