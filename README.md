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
Here are reported the visual results of the Total Least Squares for further details read the report.pdf in the repo.
![Screenshot from 2023-02-28 12-01-31](https://user-images.githubusercontent.com/94857717/221907693-51977272-fd18-42dc-9f1a-27afa7face4d.png)
