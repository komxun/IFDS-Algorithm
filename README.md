# IFDS-Algorithm
A path planning algorithm based on Interfered Fluid Dynamical System (IFDS)

# Note
- Work in-progress (Master's Thesis and AIAA submission)
- **main_static_symbolic** : Coded with symbolic maths; took very long time to run (3-4 s per obstacle)
- **main_static** : Coded numerically; very fast (~0.008 s per obstacle)

# Preliminary Results

## The effect of Shape-following parameter ($SF$)
![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/3642e2b2-43c8-4bc1-8086-a390ba167e2f)

## The effect of $\rho_{0}$
![2obj_sf1_M1](https://github.com/komxun/IFDS-Algorithm/assets/133139057/99f9778b-a29f-4107-9527-a9161be30123)
![2obj_sf1_rho10_M2](https://github.com/komxun/IFDS-Algorithm/assets/133139057/2b6cde3a-37ec-4b2b-9993-70b68ff04bc6)
![2obj_sf1_M1_topview](https://github.com/komxun/IFDS-Algorithm/assets/133139057/43215a9e-e85d-4222-94d7-75603f2e2615)
![2obj_sf1_rho10_M2_topview](https://github.com/komxun/IFDS-Algorithm/assets/133139057/9e7df2d1-838a-4e89-98d6-94f08d448b28)


## Single Static Obstacle Avoidance
![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/bb710c35-8df5-4739-958e-44b4c6e48c06)

## Multiple Static Obstacles Avoidance
![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/c6e6e666-75ed-4bfa-9605-5144fa066508)

![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/f9573a2d-f77a-43e9-bb89-db4d1d51cb8e)

![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/93cc10ae-9936-432e-8a53-565c186609f5)



# References
[1]	H. Wang, W. Lyu, P. Yao, X. Liang, and C. Liu, “Three-dimensional path planning for unmanned aerial vehicle based on interfered fluid dynamical system,” Chinese Journal of Aeronautics, vol. 28, no. 1, pp. 229–239, Feb. 2015, doi: 10.1016/J.CJA.2014.12.031.

[2]	P. Yao, H. Wang, and Z. Su, “Real-time path planning of unmanned aerial vehicle for target tracking and obstacle avoidance in complex dynamic environment,” Aerosp Sci Technol, vol. 47, pp. 269–279, Dec. 2015, doi: 10.1016/J.AST.2015.09.037.

[3]	P. Yao, H. Wang, and Z. Su, “UAV feasible path planning based on disturbed fluid and trajectory propagation,” Chinese Journal of Aeronautics, vol. 28, no. 4, pp. 1163–1177, Aug. 2015, doi: 10.1016/J.CJA.2015.06.014.

[4]	D. Celestini, S. Primatesta, and E. Capello, “Trajectory Planning for UAVs Based on Interfered Fluid Dynamical System and Bézier Curves,” IEEE Robot Autom Lett, vol. 7, no. 4, pp. 9620–9626, Oct. 2022, doi: 10.1109/LRA.2022.3191855.

[5]	P. Yao and S. Zhao, “Three-Dimensional Path Planning for AUV Based on Interfered Fluid Dynamical System under Ocean Current (June 2018),” IEEE Access, vol. 6, pp. 42914–42916, Jul. 2018, doi: 10.1109/ACCESS.2018.2861468.

[6]	Y. Wang, H. Wang, J. Wen, Y. Lun, and J. Wu, “Obstacle avoidance of UAV based on neural networks and interfered fluid dynamical system,” in Proceedings of 2020 3rd International Conference on Unmanned Systems, ICUS 2020, Institute of Electrical and Electronics Engineers Inc., Nov. 2020, pp. 1066–1071. doi: 10.1109/ICUS50048.2020.9274988.
