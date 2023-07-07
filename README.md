# Update
- [x] Added local and global optimization feature (Interim or SQP) - Enabled by `useOptimizer` variable. The optimization algorithm is done by `fmincon` built-in function
- [x] Safeguard function for rho0 - Input the minimum allowed gap distance `Rg`. The Safeguard is happening inside the `calc_ubar` function where `rho0_star` is introduced
- [x] Improved program structure (added `Param` table containing all important parameters) 

# IFDS-Algorithm
A path planning algorithm based on Interfered Fluid Dynamical System (IFDS)

# Note
- Work in-progress (Master's Thesis and AIAA submission)
- **main_static_symbolic** : Coded with symbolic maths; took very long time to run (3-4 s per obstacle)
- **main_static** : Coded numerically; very fast (~0.008 s per obstacle)

