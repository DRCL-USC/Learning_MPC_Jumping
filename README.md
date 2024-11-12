## Variable-Frequency Model Learning

This branch contains the code for learning the residual dynamic model for aggressive jumping maneuvers

Hardware Experiment Video: https://www.youtube.com/watch?v=oqF4PsurAxU

### Prerequisites
- tested with Ubuntu 20.04, Python 3.8 and Python 3.9
- torch>=2.0.0, pandas, matplotlib

- or use RAL_env.yml instead to setup environment for training:
  ``` 
  conda env create -f RAL_env.yml
  conda activate RAL_env.yml
  ```

<!-- ## Train all jumping phases residual model  -->

### Training and evaluation

* Run ```train_full.py``` to train the model
```
$ cd residual_learning
$ python3 train_full.py --total_steps=20000 --learn_rate=2e-4 --num_points=11
```
* Run evaluation
```
$ cd residual_learning
$ python3 evaluation_full.py
```
<p float="left">
<img src="/residual_learning/result/rollout.png" width="500">
</p>

* Generate a dataset for training and testing
```
$ cd residual_learning/data/mix-20traj/fmpc_pd/
$ python generate_dataset.py
```

* Convert the PyTorch model to Torch Script, then Serialize
```
$ cd residual_learning
$ python3 convert_to_hardware_full.py
```
## Real-time MPC Execution

### Prerequisites
- eigen-3.3.7
- lcm-1.4.0
- libtorch-cxx11--2.0.1 (link to download: )
- MoCap system

### Get jumping reference

```
$ cd hw_execution/a1_robot_code_jumpMPC/Controllers/optimization_data/jump2D/MDC
$ python parse_data_new.py
```

### Build and run

```
$ cd hw_exp/a1_robot_code_jumpMPC
$ mkdir build 
$ cd build 
$ cmake ..
$ make -j4
```
Note: Check the path of the downloaded libtorch library in a1_robot_code_jumpMPC/Controllers/CMakeLists.txt
```
list(APPEND CMAKE_PREFIX_PATH "/path/to/the/libtorch")
```

### Contact Information:
Chuong Nguyen -- vanchuon@usc.edu
Abdullah Altawaitan -- aaltawaitan@ucsd.edu
Thai Duong -- tduong@ucsd.edu
