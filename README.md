# This version
- [x] Added local and global optimization feature (Interim or SQP) - Enabled by `useOptimizer` variable. The optimization algorithm is done by `fmincon` built-in function
- [x] Safeguard function for rho0 - Input the minimum allowed gap distance `Rg`. The Safeguard is happening inside the `calc_ubar` function where `rho0_star` is introduced
- [x] Improved program structure (added `Param` table containing all important parameters) 

# Not finished
- [ ] Constraints matrix not introduced (e.g. Weather data)
- [ ] Fuel consumption and flight time cost not considered
- [ ] Overlapped shape problem not fixed
- [ ] Stagnation problem not fixed (e.g. when path is orthogonal to cylinder surface)
- [ ] Path following and UAV dynamics not introduced
- [ ] Evaluation method not considered

# Problem Noticed
- **Problem1**: The effect of overlapped object ruined the path planning result
- **Problem2**: The Barrier of the cylinder, cone, and parallel piped are not uniformly enclosed -> This is because of how the safeguard function is derived from sphere

# Possible Solutions
- **Problem1**: Follow the literature to solve the overlapped problem
- **Problem2**: Derive the barrier for the cylinder case, or even for the general case

# Some results
## Safeguard
![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/c02434b7-347a-4f47-9fc3-f902a708bdda)

![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/a868cfcb-5d4b-4717-abfa-fb86b59ba913)





