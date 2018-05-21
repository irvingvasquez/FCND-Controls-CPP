#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}


VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  //printf("thrust: %f  mx: %f my: %f  mz: %f \n", collThrustCmd, momentCmd.x, momentCmd.y, momentCmd.z);
  
  float a_bar = collThrustCmd;
  float b_bar = sqrt_2 * momentCmd.x / L;
  float c_bar = sqrt_2 * momentCmd.y / L;
  float d_bar = -momentCmd.z / kappa;
  
  //printf("a: %f  b: %f c: %f  d: %f \n", a_bar, b_bar, c_bar, d_bar);
  
  float f_3 = -0.25f * (c_bar - a_bar + d_bar - b_bar);
  float f_4 = -0.5f * (c_bar - a_bar + 2 * f_3);
  float f_2 = -0.5f * (b_bar-a_bar + 2*f_4);
  float f_1 = a_bar - f_2 - f_4 - f_3;
  
  //printf("f1: %f  f2: %f f3: %f  f4: %f \n", f_1, f_2, f_3, f_4);
  
  //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right
  
  if(f_1 > maxMotorThrust)
    cmd.desiredThrustsN[0] = maxMotorThrust;
  else
    if(f_1 < minMotorThrust)
      cmd.desiredThrustsN[0] = minMotorThrust;
    else
      cmd.desiredThrustsN[0] = f_1;
    
  if(f_2 > maxMotorThrust)
    cmd.desiredThrustsN[1] = maxMotorThrust;
  else
    if(f_2 < minMotorThrust)
      cmd.desiredThrustsN[1] = minMotorThrust;
    else
      cmd.desiredThrustsN[1] = f_2;

  if(f_3 > maxMotorThrust)
    cmd.desiredThrustsN[2] = maxMotorThrust;
  else
    if(f_3 < minMotorThrust)
      cmd.desiredThrustsN[2] = minMotorThrust;
    else
      cmd.desiredThrustsN[2] = f_3;
    
  if(f_4 > maxMotorThrust)
    cmd.desiredThrustsN[3] = maxMotorThrust;
  else
    if(f_4 < minMotorThrust)
      cmd.desiredThrustsN[3] = minMotorThrust;
    else
      cmd.desiredThrustsN[3] = f_4;
    
  
//   if(f_3 > maxMotorThrust)
//     f_3 = maxMotorThrust;
//   if(f_3 < minMotorThrust)
//     f_3 = minMotorThrust;
  
//   if(f_4 > maxMotorThrust)
//     f_4 = maxMotorThrust;
//   if(f_4 < minMotorThrust)
//     f_4 = minMotorThrust;
  

  //cmd.desiredThrustsN[2] = f_3; // rear left
//   cmd.desiredThrustsN[3] = f_4; // rear right
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}


V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float u_bar_p = kpPQR.x * (pqrCmd.x - pqr.x);
  float u_bar_q = kpPQR.y * (pqrCmd.y - pqr.y);
  float u_bar_r = kpPQR.z * (pqrCmd.z - pqr.z);
  //printf("pqrCmd.z: %f  pqr.z: %f \n", pqrCmd.z, pqr.z);
  
  momentCmd.x = Ixx * u_bar_p;
  momentCmd.y = Iyy * u_bar_q;
  momentCmd.z = Izz * u_bar_r;
  //printf("Error: %f  momentCmd.z: %f \n", pqrCmd.z - pqr.z, momentCmd.z);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  return momentCmd;
}



// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float c_c = -collThrustCmd / mass;
  //printf("c_c: %f  mass: %f \n", c_c, mass);
  
  float b_c_x = accelCmd.x / c_c;
  float b_c_y = accelCmd.y / c_c;
  //printf("accelCmd.x: %f  accelCmd.y: %f \n", accelCmd.x, accelCmd.y);
  //printf("b_c_x: %f  b_c_y: %f \n", b_c_x, b_c_y);
  
  if (b_c_x < - maxTiltAngle){
    b_c_x = -maxTiltAngle;
    //printf("Excede tilt %f  limit %f \n", b_c_x, -maxTiltAngle);
  }
    
  if (b_c_x > maxTiltAngle){
    //printf("Excede tilt %f  limit %f \n", b_c_x, maxTiltAngle);
    b_c_x = maxTiltAngle;
  }
  
  
  if (b_c_y < - maxTiltAngle){
    b_c_y = -maxTiltAngle;
    //printf("Excede tilt %f  limit %f \n", b_c_x, -maxTiltAngle);
  }
    
  if (b_c_y > maxTiltAngle){
    //printf("Excede tilt %f  limit %f \n", b_c_x, maxTiltAngle);
    b_c_y = maxTiltAngle;
  }
  
  float b_a_x = R(0,2);
  float b_a_y = R(1,2);
  //printf("b_a_x: %f  b_a_y: %f \n", b_a_x, b_a_y);
  
  float b_c_x_dot = kpBank * (b_c_x - b_a_x); 
  float b_c_y_dot = kpBank * (b_c_y - b_a_y);
  
  //p_c
  pqrCmd.x =  1/R(2,2) * (R(1,0)*b_c_x_dot - R(0,0)*b_c_y_dot);
  //q_c
  pqrCmd.y =  1/R(2,2) * (R(1,1)*b_c_x_dot - R(0,1)*b_c_y_dot);
  //printf("p_c: %f  q_c: %f \n", p_c, q_c);
  
  //p_c =  1/R(2,2) * (-R(1,0)*b_c_x_dot + R(0,0)*b_c_y_dot);
  //q_c =  1/R(2,2) * (-R(1,1)*b_c_x_dot + R(0,1)*b_c_y_dot);
  //printf("p_c: %f  q_c: %f \n", p_c, q_c);
  
  //pqrCmd.x = p_c;
  //pqrCmd.y = q_c;
  //pqrCmd.z = 0.0;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float z_error = posZCmd - posZ;
  float z_dot = kpPosZ * (z_error) + velZCmd;
  //printf("posZCmd: %f  posZ: %f \n", posZCmd, posZ);
  //printf("u z_dot: %f  velZ: %f \n", z_dot, velZ);
  
  if(-z_dot > maxAscentRate)
    z_dot = -maxAscentRate;
  else
    if(z_dot > maxDescentRate)
      z_dot = maxDescentRate;
  
  //printf("z_dot: %f  velZ: %f \n", z_dot, velZ);
  integratedAltitudeError += z_error * dt;
  float z_dot_dot = kpVelZ * (z_dot - velZ) + KiPosZ * (integratedAltitudeError) +  accelZCmd;
  //printf("z_dot_dot: %f  accelZCmd: %f \n", z_dot_dot, accelZCmd);
  
  thrust = (CONST_GRAVITY - z_dot_dot) * mass / R(2,2);
  //thrust = thrust * mass;
  //printf("u z_dot_dot: %f  thrust: %f \n", z_dot_dot, thrust);
  
  if (thrust > maxMotorThrust*4.0)
    thrust = maxMotorThrust*4.0;
  
  if (thrust < minMotorThrust*4.0)
     thrust = minMotorThrust*4.0;
  
  //printf("z_dot_dot: %f  thrust: %f \n", z_dot_dot, thrust);
  //printf("thrust: %f  accelZCmd: %f \n", thrust, accelZCmd);
  
  //thrust = 0.0;
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //TODO:
  float u_vel_x = kpPosXY * (posCmd.x - pos.x) + velCmd.x;
  float u_vel_y = kpPosXY * (posCmd.y - pos.y) + velCmd.y;
  //printf("posCmd.x: %f  pos.x: %f velCmd.x %f \n", posCmd.x, pos.x, velCmd.x);
  //printf("posCmd.y: %f  pos.y: %f velCmd.y %f \n", posCmd.y, pos.y, velCmd.y);
  
  if(u_vel_x > maxSpeedXY)
    u_vel_x = maxSpeedXY;
  if(u_vel_y > maxSpeedXY)
    u_vel_y = maxSpeedXY;
    if(u_vel_x < -maxSpeedXY)
    u_vel_x = -maxSpeedXY;
  if(u_vel_y < -maxSpeedXY)
    u_vel_y = -maxSpeedXY;
  
 // printf("u_vel_x: %f  \n", u_vel_x);
  
  float u_acc_x = kpPosXY * (posCmd.x - pos.x) + kpVelXY * (u_vel_x - vel.x);
  float u_acc_y = kpPosXY * (posCmd.y - pos.y) + kpVelXY * (u_vel_y - vel.y);
  
  //printf("u_vel_x: %f  vel.x %f u_acc_x %f \n", u_vel_x, vel.x, u_acc_x);
  //printf("u_vel_y: %f  vel.y %f u_acc_y %f \n", u_vel_y, vel.y, u_acc_y);
  
  accelCmd.x += u_acc_x;
  accelCmd.y += u_acc_y;
  
  if(accelCmd.x > maxAccelXY)
    accelCmd.x = maxAccelXY;
  
  if(accelCmd.y > maxAccelXY)
    accelCmd.y = maxAccelXY;
  
  //accelCmd = accelCmdFF;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //printf("yawCmd: %f  yaw: %f \n", yawCmd, yaw);
  float yawCmd_range = fmod(yawCmd, 2*3.1416);
  float yaw_range = fmod(yaw, 2*3.1416);
  //printf("yawCmd_r: %f  yaw_r: %f \n", yawCmd_range, yaw_range);
  //float error = ;

  yawRateCmd = kpYaw * (yawCmd - yaw);
  //printf("error: %f  yawRateCmd: %f \n", yawCmd - yaw, yawRateCmd);  
  
  //yawRateCmd = -0.1;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;
}


VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
