## Variable-Frequency Model Learning

This branch contains the code for learning the residual dynamic model for aggressive jumping maneuvers

Hardware Experiment Video: https://www.youtube.com/watch?v=oqF4PsurAxU

<!-- ## Train all jumping phases residual model  -->

* Run ```train_full.py``` to train the model
```
$ cd residual_learning
$ python3 train_full.py --total_steps=20000 --learn_rate=2e-4 --num_points=11
```
* To run evaluation
```
$ cd residual_learning
$ python3 evaluation_full.py
```
<p float="left">
<img src="/residual_learning/result/rollout.png" width="500">
</p>

* Generate a dataset 
```
$ cd residual_learning/data/mix-20traj/fmpc_pd/
$ python generate_dataset.py
```

## Contact Information:
Chuong Nguyen -- vanchuon@usc.edu
Abdullah Altawaitan -- aaltawaitan@ucsd.edu
Thai Duong -- tduong@ucsd.edu
