// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {

  //motor init
  private final Field2d m_field = new Field2d();

  public final WPI_TalonFX rightLead = new WPI_TalonFX(Constants.MotorID.kRightDriveLead);
  public final WPI_TalonFX rightFollow = new WPI_TalonFX(Constants.MotorID.kRightDriveFollow);
  public final WPI_TalonFX leftLead = new WPI_TalonFX(Constants.MotorID.kLeftDriveLead);
  public final WPI_TalonFX leftFollow = new WPI_TalonFX(Constants.MotorID.kLeftDriveFollow);

  public final WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(4);
  public final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d());


  public final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLead, leftFollow);
  public final MotorControllerGroup rightMotors = new MotorControllerGroup(rightLead, rightFollow);


  //drivetrain kinematics init 
  private static DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters);


  //PID things init
  private static final PIDController m_leftPIDController = new PIDController(Constants.DriveConstants.kDrivekP, Constants.DriveConstants.kDrivekI, Constants.DriveConstants.kDrivekD);
  private static final PIDController m_rightPIDController = new PIDController(Constants.DriveConstants.kDrivekP, Constants.DriveConstants.kDrivekI, Constants.DriveConstants.kDrivekD);
  private double rotSpeed = 0;
  private double xSpeed = 0;
  
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(10);


  //feedforward init
  private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.kDriveKS, Constants.DriveConstants.kDriveKV, Constants.DriveConstants.kDriveKA);





  public DriveSubsystem() {
    leftFollow.follow(leftLead);
    rightFollow.follow(rightLead);
    rightMotors.setInverted(true);
    rightLead.setSelectedSensorPosition(0);
    leftLead.setSelectedSensorPosition(0);
  }

  public double getHeading() {
    return pigeon2.getRotation2d().getDegrees();
  }

  public void drive(double fwd, double rot, boolean speedDriveStat){
    //deadband + calculate speeds

    //Drive forward
    if (speedDriveStat)
    {
      if(Math.abs(fwd) < 0.1){ //In deadband
        xSpeed = 0;
      }
      else{ //Outside of deadband
        xSpeed = -speedLimiter.calculate(fwd) *  Constants.DriveConstants.kMaxFastDriveSpeed;
      }
  
      //Drive twist
      if(Math.abs(rot) < 0.1){ //In deadband
        rotSpeed = 0;
      }
      else{ //Outside of deadband
         rotSpeed = -rotLimiter.calculate(rot) * Constants.DriveConstants.kMaxFastAngularSpeed;
      }

      // SmartDashboard.putBoolean("Speed Drive", true);
    }
    else if(!speedDriveStat)
    {
      if(Math.abs(fwd) < 0.1){ //In deadband
        xSpeed = 0;
      }
      else{ //Outside of deadband
        xSpeed = -speedLimiter.calculate(fwd) *  Constants.DriveConstants.kMaxDriveSpeed;
      }
  
      //Drive twist
      if(Math.abs(rot) < 0.1){ //In deadband
        rotSpeed = 0;
      }
      else{ //Outside of deadband
         rotSpeed = -rotLimiter.calculate(rot) * Constants.DriveConstants.kMaxAngularSpeed;
      }
      // SmartDashboard.putBoolean("Speed Drive", true);

    }


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

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);

  }
  
  public void zeroSensors() {
    rightLead.setSelectedSensorPosition(0);
    leftLead.setSelectedSensorPosition(0);
    rightFollow.setSelectedSensorPosition(0);
    leftFollow.setSelectedSensorPosition(0);

    pigeon2.reset();
    
}

  //convert sensorPosition to meters
  public static double toMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / 2048;
    double wheelRotations = motorRotations / 7.44;
    double positionMeters = wheelRotations * (Math.PI * Units.inchesToMeters(4));
    return positionMeters;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("R Drive", -(toMeters(rightLead.getSelectedSensorVelocity()) * 10));
		SmartDashboard.putNumber("L Drive", (toMeters(leftLead.getSelectedSensorVelocity()) * 10));

    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putData("Field", m_field);
    //m_odometry.update(pigeon2.getRotation2d(), leftLead.getSelectedSensorPosition(), (rightLead.getSelectedSensorPosition()));

    m_odometry.update(pigeon2.getRotation2d(), toMeters(leftLead.getSelectedSensorPosition()), toMeters(rightLead.getSelectedSensorPosition()));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    double lMeterPerSecond = toMeters(leftLead.getSelectedSensorVelocity()) * 10;

    double rMeterPerSecond = toMeters(rightLead.getSelectedSensorVelocity()) * 10;
    
    return new DifferentialDriveWheelSpeeds(lMeterPerSecond, rMeterPerSecond);
  }
}
