# Probabilistic robotics project "Planar monocular SLAM".
Final project of the probabilistic robotics course of the a.y. 2022/2023 held by professor Giorgio Grisetti. 

The aim of this project is to solve a SLAM system by developing a Total Least Square algorithm. The scenario is the following: a differential drive robot moving in a 2D environment sensing the projection of landmarks by a monocular camera.

Input :
 - Wheeled odometry
 - Camera parameters (extrinsics and intrinsics)
 - Stream of point projections with the real "id"

Output :
 - Trajectory of the robot
 - Position of the 3D landmarks
 - Error values
 
 To run the code clone the repository and run on the terminal the command
 ```
$ octave main.m

```
Here are reported only the visual results of the Total Least Square for further details read the report.pdf in the repo.
