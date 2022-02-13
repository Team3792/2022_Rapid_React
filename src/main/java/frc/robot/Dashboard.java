// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cameraserver.CameraServer;



public final class Dashboard {

  double setPointVal;

  public static void updateVals()
  {
   ShooterPID.setpoint();


  }

  public static void showVals()
  {




  }

  public static void showFeed()
  {
    CameraServer.getInstance().startAutomaticCapture(1);

  }







}
