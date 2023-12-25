# Distributed-formation-control-of-manipulators-end-effectors
In this work, we simulate the controller proposed by [1], which provides a distributed formation control of manipulators' end-effectors with internal model-based disturbance rejection. The formation control objective is achieved by assigning virtual springs between end-effectors, by adding damping terms at joints and by incorporating internal model-based dynamic compensators to counteract the effect of the disturbances.

The simulation results match with the results shown in the related work.

Additionally, extended observers were inserted in the system to estimate the unknown nonlinearity of the robots to consider the case where the parameters of the robots are unknown. This extended observer was proposed in [2] where the studied case is a single robot. However, here we consider the application of the observer for formation control problems where the simulation results show the convergence of the end-effectors toward the desired formation despite the unknown nonlinearity.

Simulink model for known parameter case:

![ssssssssssssssssssssssssssss](https://github.com/SokratALDARMINI/Distributed-formation-control-of-manipulators-end-effectors/assets/95107709/f403cb47-5804-46c5-8cd9-20073077ee71)


Simulink model for unknown parameter case:

![sssssssssssssssssss2](https://github.com/SokratALDARMINI/Distributed-formation-control-of-manipulators-end-effectors/assets/95107709/ee17b435-4ba6-4468-b173-e9ac4f0cfe52)


Simulation results with known parameters case:

![image](https://github.com/SokratALDARMINI/Distributed-formation-control-of-manipulators-end-effectors/assets/95107709/c456ef20-3a50-4fb5-87f3-2abdff3123c3)


Simulation results with unknown parameters case:

![image](https://github.com/SokratALDARMINI/Distributed-formation-control-of-manipulators-end-effectors/assets/95107709/7f383646-138f-40a6-986f-3d084cbf4d60)


[1] Wu, Haiwen, et al. "Distributed formation control of manipulators' end-effector with internal model-based disturbance rejection." arXiv preprint arXiv:2103.14595 (2021).

[2] Freidovich, Leonid B., and Hassan K. Khalil. "Performance recovery of feedback-linearization-based designs." IEEE Transactions on Automatic Control 53.10 (2008): 2324-2334.


