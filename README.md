# IMU preintegration with Symforce

This repository provides an implementation of a factor graph with IMU preintegration in symforce (C++). We use GPS measurements for pose factors, the symforce/slam ImuFactor for preintegration and between factors for Accelerometer and Gyroscope bias. In createFactors.h we provide the implementation for adding the factors.
The first timestep also has a prior factor on pose, velocity, and accelometer/gyroscope bias.

You can read more about the IMU preintegration in symforce [here](https://symforce.org/api-cpp/file/imu__factor_8h.html)

# KITTI dataset

<img src="assets/kitti_trajectory.png" alt="alt text" width="299" />

To verify the IMU preintegration in Symforce we test it on the well-known KITTI dataset and closely replicate the IMU preintegration example from [GTSAM](https://github.com/borglab/gtsam/blob/develop/examples/IMUKittiExampleGPS.cpp). We skip GPS measurements such that we have 10 poses with an ImuFactor for every measured pose, but optimize the pose at every time step to retrieve IMU rate estimates. Prior factors are based on the first GPS measurement and IMU metadata, see 'initializeValuesKITTI' for more info. We follow the GTSAM example and set the ACCEL_BIAS_DIAG_SQRT_INFO to be one over the square root of number of IMU measurements times the accelerometer/gyroscope bias sigma. While GTSAM sets a fixed TIME_DELTA for IMU measurements we set it directly from the measurements for that pose, but the difference here will in this example be negligible. Note that GTSAM uses the precision matrix (information matrix) while symforce uses the square root of this. We include a function to print the values at a particular pose identical to the one provided in GTSAM for easy comparison, see below.

```plaintext
State at #470
Pose:
R: [
   0.4006 -0.9147  0.0533;
   0.9161  0.4009 -0.0039;
  -0.0178  0.0504  0.9986
]
t: 37.6308 76.8252  0.4194
Velocity:
 5.2701 10.0252 -0.1149
Bias:
acc = -0.0068  0.0140 -0.0020 gyro =  0.0002 -0.0001 -0.0002
AccelerometerBias sqrt info at 469: 598.8024 598.8024 598.8024
GyroscopeBias sqrt info at 469: 34364.2612 34364.2612 34364.2612
Optimized pose at 469 is 37.6308 76.8252  0.4194, gps measured pose is 37.9004 73.8345  0.6205
```

Sample IMU and GPS measurements from KITTI are displayed below:

```plaintext
[2024-11-18 11:54:11.873] [info] IMU metadata: 0 0 0 0 0 0 0.01 0.000175 0 0.000167 2.91e-06 0.0100395199348279
Number of GPS measurements: 470
Sample GPS Measurements:
      Time   Position (X)   Position (Y)   Position (Z)
46534.4784        -6.8269       -11.8682         0.0403
46537.3880         3.8971         7.5451         0.0248
46538.3878         8.0789        15.6420         0.0298

Sample IMU Measurements:
      Time      Accel (X)      Accel (Y)      Accel (Z)     AngVel (X)     AngVel (Y)     AngVel (Z)
46534.4784         1.7115         0.1718         9.8053        -0.0032         0.0312        -0.0064
46536.3980         0.8342         0.6852        10.0984         0.0062         0.0075         0.0190
46536.4080         0.8658         0.6865        10.0328         0.0060         0.0070         0.0176
```

# Installation

To run the example, you can build locally with cmake or use Docker. The provided docker-compose.yml file sets up the necessary environment for running the IMU preintegration example.

```
xhost +
docker-compose down && docker-compose up --build
```

## Related Publications:

- Forster C, Carlone L, Dellaert F, et al. **On-Manifold Preintegration for Real-Time Visual-Inertial Odometry**. IEEE Transactions on Robotics, 2017, 33(1): 1-21. **[PDF](http://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)**.
