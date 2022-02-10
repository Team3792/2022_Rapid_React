// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;



import com.ctre.phoenix.motorcontrol.can.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {

  //motor init
  public final WPI_TalonFX rightLead = new WPI_TalonFX(0);
  public final WPI_TalonFX rightFollow = new WPI_TalonFX(0);
  public final WPI_TalonFX leftLead = new WPI_TalonFX(0);
  public final WPI_TalonFX leftFollow = new WPI_TalonFX(0);


  public final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLead, leftFollow);
  public final MotorControllerGroup rightMotors = new MotorControllerGroup(rightLead, rightFollow);


  //drivetrain kinematics init 
  public static final double driveTrainWidthMeters = 0.6216331403;
  private static DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(driveTrainWidthMeters);


  //PID things init
  private static final PIDController m_leftPIDController = new PIDController(0.00691, 0, 0);
  private static final PIDController m_rightPIDController = new PIDController(0.00691, 0, 0);
  private double rotSpeed = 0;
  private double xSpeed = 0;
  
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(10);
  private double kMaxSpeed = 3.0; // meters per second
  private double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second


  //feedforward init
  private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.694, 1.71, 0.367);




  public DriveSubsystem() {
    leftFollow.follow(leftLead);
    rightFollow.follow(rightLead);
    rightMotors.setInverted(true);    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double fwd, double rot){
    //deadband + calculate speeds

    // //Drive forward
    // if(Math.abs(fwd) < 0.1){ //In deadband
    //   xSpeed = 0;
    // }
    // else{ //Outside of deadband
       xSpeed = -speedLimiter.calculate(fwd) * kMaxSpeed;
    // }

    // //Drive twist
    // if(Math.abs(rot) < 0.2){ //In deadband
    //   rotSpeed = 0;
    // }
    // else{ //Outside of deadband
       rotSpeed = -rotLimiter.calculate(rot) * kMaxAngularSpeed;
    // }


    //create a wheelSpeeds object using linear and angular speed
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rotSpeed));
    setSpeeds(wheelSpeeds);

  }

  //set voltage on both sides of the robot using PID + feedforward
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
    
    final double leftOutput = m_leftPIDController.calculate(toMeters(leftLead.getSelectedSensorVelocity()), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(toMeters(-rightLead.getSelectedSensorVelocity()), speeds.rightMetersPerSecond);



    leftMotors.setVoltage(leftOutput + leftFeedforward);
    rightMotors.setVoltage(rightOutput + rightFeedforward);
  }

  //convert sensorVelocity to meters
  public static double toMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / 2048;
    double wheelRotations = motorRotations / 7.44;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(3));
    return positionMeters;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
