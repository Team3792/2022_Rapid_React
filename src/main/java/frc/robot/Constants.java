// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ButtonConstant
  {
    //Driver Joystick
    public static final int kDriveJoystick = 0;

    public static final int kAlertShoot = 5;
    

    //Operator Controller
    public static final int kOperateController = 1;
    public static final int kRunFeederButton = 4;


  }

  public static final class MotorID
  {
    //Drive Train (Talon FX/Falcon)
    public static final int kLeftDriveLead = 0;
    public static final int kLeftDriveFollow = 1;
    public static final int kRightDriveLead = 2;
    public static final int kRightDriveFollow = 3;
    

    //Shooter Motor (Talon FX/Falcon)
    public static final int kShootMotor = 10;
    //Feeder Motor (Talon FX/Falcon)
    public static final int kFeedMotor = 11;

    //Intake Motor (775)
    public static final int kIntakeMotor = 20;

  }

  public static final class DriveConstants
  {
    //Drive Kinematics
    public static final double kDriveTrainWidthMeters = 0.6216331403;

    //Drivetrain PID Vars
    public static final double kDrivekP = 0.00691;
    public static final double kDrivekI = 0;
    public static final double kDrivekD = 0;

    //Drive Speed Constants
    public static final double kMaxDriveSpeed = 3.0;           // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second


  }
  public static final class ShooterConstants
  {
    //Shooter PID constants
    public static final double shooterkP = 0.002;
    public static final double shooterkI = 0;
    public static final double shooterkD = 0;

    //FF constants
    public static final double shooterKs = 0.0015;
    public static final double shooterKv = 0.00325;
    public static final double shooterKa = 0.003;

  }

  public static final class ClimbConstants
  {

    //Climb PID constants
    public static final double kClimbkP = 0;
    public static final double kClimbkI = 0;
    public static final double kClimbkD = 0;

     //Arm PID constants
     public static final double kArmkP = 0;
     public static final double kArmkI = 0;
     public static final double kArmkD = 0;


  }







}
