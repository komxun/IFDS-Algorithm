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

