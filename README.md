Version for IRP Dynamic Autorouting\
Results Video: https://youtu.be/XtmcNa-w4-0?si=V0FAj7HmrgcvlQuK

[![Watch the video](![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/27566bef-577c-4077-8ae8-19a39de23594)
](https://youtu.be/XtmcNa-w4-0?si=V0FAj7HmrgcvlQuK)



# This version
- [x] Constraints matrix has been introduced (e.g. Weather data)
- [x] Path following and UAV dynamics have been introduced
- [x] Evaluation methods are considered (e.g. real-time performance)

# Not finished
- [ ] Fuel consumption and flight time cost not considered
- [ ] Overlapped shape problem not fixed
- [ ] Stagnation problem not fixed (e.g. when path is orthogonal to cylinder surface)


# Problem Noticed
- **Problem1**: The effect of overlapped object ruined the path planning result
- **Problem2**: The Barrier of the cylinder, cone, and parallel piped are not uniformly enclosed -> This is because of how the safeguard function is derived from sphere
- **Problem3**: After restructuring the code, the Global Optimizer ran a lot slower (but maybe more accurate than `verion2_legacy` since the objective function considers the SafeGuard and identical to the IFDS algorithm called in the main file)

# Possible Solutions
- **Problem1**: Follow the literature to solve the overlapped problem
- **Problem2**: Derive the barrier for the cylinder case, or even for the general case






