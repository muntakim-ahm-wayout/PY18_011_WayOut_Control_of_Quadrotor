# The C++ Project Readme #

This is the readme for the C++ project.
Theory figures
<img src="drone1_1.png"
     alt="drone"
     style="float: left; margin-right: 10px;" />
<img src="Drone2.png"
     alt="drone"
     style="float: left; margin-right: 10px;" />
<img src="control1_mod.png"
     alt="Controller"
     style="float: left; margin-right: 10px;" />
 <img src="control2.png"
     alt="Controller"
     style="float: left; margin-right: 10px;" />    
     

## scenario 1

Modify mass = 0.5

<p align="center">
<img src="assets/Senario 1.gif" width="500"/>
</p>


<img src="assets/senario 1.PNG"
     alt="senario 1"
     style="float: left; margin-right: 10px;" />


### Body rate and roll/pitch control (scenario 2) ###



1. Implement body rate control

 - implement the code in the function `GenerateMotorCommands()`
Rubrics item: Implement calculating the motor commands given commanded thrust and moments in C++.
 ```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right
  float l = L/sqrt(2.0f); //Find distance from body center to wing
  //Back calculating
  float force_from_thrust_cmd=collThrustCmd; // = rotor 1 thrust + rotor 2 thrust + rotor 3 thrust + rotor 4 thrust
  float force_from_moment_x = momentCmd.x/l; // = rotor 1 thrust - rotor 2 thrust + rotor 3 thrust - rotor 4 thrust
  float force_from_moment_y = momentCmd.y/l; // = rotor 1 thrust + rotor 2 thrust - rotor 3 thrust - rotor 4 thrust
  float force_from_moment_z = -momentCmd.z/kappa; //Downward positive // = rotor 1 thrust - rotor 2 thrust - rotor 3 thrust + rotor 4 thrust
  
  cmd.desiredThrustsN[0] = (force_from_thrust_cmd+force_from_moment_x+force_from_moment_y+force_from_moment_z)/4.0f; // front left
  cmd.desiredThrustsN[1] = (force_from_thrust_cmd-force_from_moment_x+force_from_moment_y-force_from_moment_z)/4.0f; // front right
  cmd.desiredThrustsN[2] = (force_from_thrust_cmd+force_from_moment_x-force_from_moment_y-force_from_moment_z)/4.0f; // rear left
  cmd.desiredThrustsN[3] = (force_from_thrust_cmd-force_from_moment_x-force_from_moment_y+force_from_moment_z)/4.0f; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////
 ```
 - implement the code in the function `BodyRateControl()`
Rubrics Item: Implemented body rate control in C++.

 ```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F moment_3_iniertia = V3F(Ixx, Iyy, Izz);
  V3F prq_error = pqrCmd - pqr;
  V3F m_c = kpPQR*prq_error;
  momentCmd=moment_3_iniertia*m_c;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
 ```
 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot
 ```
 # Angle rate gains
kpPQR = 92, 92, 5
```


2. Implement roll / pitch control
We won't be worrying about yaw just yet.

 - implement the code in the function `RollPitchControl()`
  Rubrics item: Implement roll pitch control in C++.
 ```
   if (collThrustCmd>0)
  {
	  float c_d = -collThrustCmd / mass ;
	  // b_x_c
	  float R13_target = CONSTRAIN(accelCmd.x/c_d,-maxTiltAngle,maxTiltAngle);
	  // b_y_c
	  float R23_target = CONSTRAIN(accelCmd.y/c_d,-maxTiltAngle,maxTiltAngle);
	  
	  float R13_actual = R(0,2);
	  float R23_actual = R(1,2);
	  
	  float b_x_p_term=(R13_target-R13_actual)*kpBank;
	  float b_y_p_term=(R23_target-R23_actual)*kpBank;
	  
	  pqrCmd.x = (R(1,0)*b_x_p_term-R(0,0)*b_y_p_term)/R(2,2);
	  pqrCmd.y = (R(1,1)*b_x_p_term-R(0,1)*b_y_p_term)/R(2,2);
  }
  else
  {
	  pqrCmd.x=0.0;
	  pqrCmd.y=0.0;
  }
 ```
 - Tune `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot
 
 ```
 kpBank = 15
 ```

<p align="center">
<img src="assets/Senario 2.gif" width="500"/>
</p>
 - scenario 2
   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds
<img src="assets/senario 2.png"
     alt="senario 2"
     style="float: left; margin-right: 10px;" />

### Position/velocity and yaw angle control (scenario 3) ###

Next, you will implement the position, altitude and yaw control for your quad.  For the simulation, you will use `Scenario 3`.  This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

 - implement the code in the function `LateralPositionControl()`
 Rubrics Item: Implement lateral position control in C++.
 ```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  if (velCmd.mag()>maxSpeedXY)
  {
	  velCmd=velCmd.norm()*maxSpeedXY;
	  
	  accelCmd = kpPosXY*(posCmd-pos)+kpVelXY*(velCmd-vel) + accelCmd;
	  if (accelCmd.mag()>maxAccelXY)
	  {
		  accelCmd = accelCmd.norm()*maxAccelXY;
	  }
	  
  } 

  /////////////////////////////// END STUDENT CODE ////////////////////////////
 ```
 - implement the code in the function `AltitudeControl()`
 Rubrics item: Implement altitude controller in C++.
 From Scenario 4 -->add basic integral control to help with the different-mass vehicle
 This also incorporated
 ```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float posZ_error=posZCmd-posZ;
  float velZ_error=velCmd-velZ;
  cumalative_error +=  posZ_error*dt;
  
  float b_z =R(2,2);
  float u_1_bar = posZ_error*kpPosZ + cumalative_error*KiPosZ + velZ_error*kpVelZ + accelZCmd
  
  float acceler = (u_1_bar - CONST_GRAVITY)/b_z;
  
  acceler = CONSTRAIN(acceler, -maxAscentRate/dt, maxAscentRate/dt);
  thrust=-acceler*mass //Downward positive



  /////////////////////////////// END STUDENT CODE ////////////////////////////
 ```
 - tune parameters `kpPosZ` and `kpPosZ`
 ```
 # Position control gains
kpPosXY = 30
kpPosZ = 20
 ```
 - tune parameters `kpVelXY` and `kpVelZ`
 ```
 # Velocity control gains
kpVelXY = 14
kpVelZ = 14
 ```



 - implement the code in the function `YawControl()`
 Rubrics item: Implement yaw control in C++.
 ```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  if (yawCmd>0)
  {
	  yawCmd = fmodf(yawCmd,2.f*F_PI);
  }
  else
  {
	  yawCmd = fmodf(yawCmd,-2.f*F_PI);
  }
  float yaw_error = yawCmd-yaw;
  if (yaw_error>F_PI)
  {
	  yaw_error=yaw_error-2.f*F_PI;
  }
  if (yaw_error<-F_PI)
  {
	  yaw_error=yaw_error+2.f*F_PI;
  }
  yawRateCmd=kpYaw*yaw_error;
	  


  /////////////////////////////// END STUDENT CODE ////////////////////////////
 ```
 - tune parameters `kpYaw` and the 3rd (z) component of `kpPQR`
 
 ```
 # Angle rate gains
kpPQR = 92, 92, 20
 ```


<p align="center">
<img src="assets/Senario 3.gif" width="500"/>
</p>
 - scenario 3
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second
<img src="assets/senario 3.png"
     alt="senario 3"
     style="float: left; margin-right: 10px;" />

**Hint:**  For a second order system, such as the one for this quadcopter, the velocity gain (`kpVelXY` and `kpVelZ`) should be at least ~3-4 times greater than the respective position gain (`kpPosXY` and `kpPosZ`).

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).
```
# Position control gains
kpPosXY = 30
kpPosZ = 20
KiPosZ = 40

# Velocity control gains
kpVelXY = 13
kpVelZ = 9
```

2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.
```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float posZ_error=posZCmd-posZ;
  float velZ_error=velCmd-velZ;
  cumalative_error +=  posZ_error*dt;
  
  float b_z =R(2,2);
  float u_1_bar = posZ_error*kpPosZ + cumalative_error*KiPosZ + velZ_error*kpVelZ + accelZCmd
  
  float acceler = (u_1_bar - CONST_GRAVITY)/b_z;
  
  acceler = CONSTRAIN(acceler, -maxAscentRate/dt, maxAscentRate/dt);
  thrust=-acceler*mass //Downward positive



  /////////////////////////////// END STUDENT CODE ////////////////////////////
  ```

3. Tune the integral control, and other control parameters until all the quads successfully move properly. 
```
KiPosZ = 40
```

<p align="center">
<img src="assets/Senario 4.gif" width="500"/>
</p>
 - scenario 4
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds
<img src="assets/senario 4.png"
     alt="senario 4"
     style="float: left; margin-right: 10px;" />

### Tracking trajectories ###

Now that we have all the working parts of a controller, you will put it all together and test it's performance once again on a trajectory.  For this simulation, you will use `Scenario 5`.  This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.  For those interested in seeing how you might be able to improve the performance of your drone by adjusting how the trajectory is defined, check out **Extra Challenge 1** below!

How well is your drone able to follow the trajectory?  It is able to hold to the path fairly well?
<p align="center">
<img src="assets/Senario 5.gif" width="500"/>
</p>
 - scenario 5
   - position error of the quad should be less than 0.25 meters for at least 3 seconds
<img src="assets/senario 5.png"
     alt="senario 5"
     style="float: left; margin-right: 10px;" />




## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
