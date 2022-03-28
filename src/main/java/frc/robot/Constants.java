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

  public static String ballType = "red";

  public static int stateCounter = 0;

  public static final class ButtonConstant
  {
    //Driver Joystick
    public static final int kDriveJoystick = 0;

    public static final int kAlertShoot = 5;

    public static final int targetAlignButton = 3;
    public static final int krevIntakeButton = 4;
    

    //Operator Controller
    public static final int kOperateController = 1;
    public static final int kRunFeederButton = 4;

    public static final int kXButton = 1;
    public static final int kCircleButton = 2;
    public static final int kSquareButton = 3;
    public static final int kTriangleButton = 4;

    public static final int kL1Button = 5;
    public static final int kR1Button = 6;


    public static final int kLXAxis = 0;
    public static final int kLYAxis = 1;

    public static final int kLTrigger = 2;
    public static final int kRTrigger = 3;

    public static final int kRXAxis = 4;
    public static final int kRYAxis = 5;


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
    //Feeder Motor (Victor SPX)
    public static final int kFeedMotor = 11;

    //Back Roller (Talon SRX)
    public static final int kRollerMotor = 12;


    //Intake Motor (Victor SPX)
    public static final int kIntakeMotor = 20;

    //Elevator Motor (Talon FX/Falcon)
    public static final int kLeftElevatorMotor = 30;
    public static final int kRightElevatorMotor = 31;

    //Elevator Motor (Talon FX/Falcon)
    public static final int kLeftArmMotor = 32;
    public static final int kRightArmMotor = 33;

  }

  public static final class DriveConstants
  {
    //Drive Kinematics
    public static final double kDriveTrainWidthMeters = 0.71;

    //Drivetrain PID Vars
    public static final double kDrivekP = .25;
    public static final double kDrivekI = 0;
    public static final double kDrivekD = 0;

    //Drive Speed Constants
    public static final double kMaxDriveSpeed = 2;           // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
    public static final double kMaxAcceleration = 1;

    public static final double kMaxFastDriveSpeed = 3.5;           // meters per second
    public static final double kMaxFastAngularSpeed = 3 * Math.PI; // one rotation per second


    public static final double kDriveKS = 0.7188;
    public static final double kDriveKV = 2.4646;
    public static final double kDriveKA = 1.0952;


    public static final double kTurnRateToleranceDeg = 2;
    public static final double kTurnRateToleranceDegPerS = 0.5;
  }

  public static final class ShooterConstants
  {
    //Shooter PID constants

    /**
     * Which PID slot to pull gains from.
     */
    public static final int kSlotIdx = 0;

    /**
     * Talon FX supports multiple (cascaded) PID loops, we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    /**
     * Set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 30;

    /**
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     * 
     */
    public static final double kshooterP = 0.2;
    public static final double kshooterI = 0;
    public static final double kshooterD = 2;
    public static final double kshooterF = 1023.0/20660.0;
    public static final double kshooterIz = 300;
    public static final double kshooterPeakOut = 1.00;
  }

  public static final class RollerConstants
  {
    /**
     * Which PID slot to pull gains from.
     */
    public static final int kSlotIdx = 0;

    /**
     * Talon SRX/ Victor SPX supports multiple PID loops, we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    /**
     * Set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 30;

    /**
     * PID Gains
     * 
    */
    // Feedforward = 869.55/124552.0, 85% (given 100% = 1023) power goes 124552 u/100ms (bad!(?))
    public static final double krollerP = 0.001;
    public static final double krollerI = 0.00001;
    public static final double krollerD = 0;
    public static final double krollerF = 0.005;
    public static final double krollerIz = 20000;
    public static final double krollerPeakOut = 1.00;
  }

  public static final class ElevatorConstants
  {
    //Elevator PID constants
    public static final double kElevatorP = 0.05;
    public static final double kElevatorI = 0.0005;
    public static final double kElevatorD = 0;
    public static final double kElevatorF = 0.06;
    public static final double kElevatorIzone = 3000;
    public static final double kElevatorPeakOutput = 1;

    public static final double setpointUp = 300000;
    
    public static final double setpointDown = 0;

    //Motion Magic Constants
    public static final double kElevatorMaxV = 21000; // u/100ms
    public static final double kElevatorAccel = 150000; // u/100ms/s
  }

  public static final class ElevatorAuxConstants
  {
    //Elevator AUX PID Constants
    public static final double kElevatorAuxP = 0.001;
    public static final double kElevatorAuxI = 0;
    public static final double kElevatorAuxD = 0;
    public static final double kElevatorAuxF = 0;
    public static final double kElevatorAuxIzone = 0;
    public static final double kElevatorAuxPeakOutput = 0.8;
  }

  public static final class ClimbConstants
  {
    //Climb PID constants
    public static final double kClimbP = 0.22478;
    public static final double kClimbI = 0;
    public static final double kClimbD = 0;
    public static final double kClimbF = 0.04809    ;
    public static final double kClimbIzone = 50;
    public static final double kClimbPeakOutput = 0.1;

    public static final double setpointForward = 300;
    public static final double setpointBack = 0;

    //Motion Magic Constants
    public static final double kClimbMaxV = 114; // u/100ms
    public static final double kClimbAccel = 228; // u/100ms/s
  }

  public static final class ClimbAuxConstants
  {
    //Climb AUX PID Constants
    public static final double kClimbAuxP = 0;
    public static final double kClimbAuxI = 0;
    public static final double kClimbAuxD = 0;
    public static final double kClimbAuxF = 0;
    public static final double kClimbAuxIzone = 0;
    public static final double kClimbAuxPeakOutput = 0.75;
  }

  public static final class PathConstants
  {
  }

  public static class GlobalStateConstants
  {
    public static boolean kRingLightState = false;  //Default False
    public static boolean kServoState = true; //Default True
    
  }

}
