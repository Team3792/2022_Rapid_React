// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutoRoutines;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.RollerCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.Autonomous.AutoCommands.TaxiCmd;
import frc.robot.commands.Autonomous.AutoCommands.AutoDriveBackMore;
import frc.robot.commands.Autonomous.AutoCommands.AutoFeedCmd;
import frc.robot.commands.Autonomous.SemiAuto.TurnToAngleCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;




/** default drive using the DriveSubystem. */
public class Auto4BallCenter extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final DriveSubsystem driveTrain;
  private final IntakeSubsystem intake;
  private final FeedSubsystem feeder;
  private final ShooterSubsystem shooter;
  private final RollerSubsystem roller;


  
  public String trajectoryGetBallJSON = "paths/output/4ball.wpilib.json";
  public Trajectory trajectoryGetBall = new Trajectory();

  public String trajectoryBackJSON = "paths/output/4ballBack.wpilib.json";
  public Trajectory trajectoryBack = new Trajectory();

  //values for controlling stages
  boolean driveDone;
  boolean shootDone;



 
  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public Auto4BallCenter(DriveSubsystem driveTrain, IntakeSubsystem intake, FeedSubsystem feeder, ShooterSubsystem shooter, RollerSubsystem roller) {
    this.driveTrain = driveTrain;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;
    this.roller = roller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryGetBallJSON);
      trajectoryGetBall = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryGetBallJSON, ex.getStackTrace());
    }

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryBackJSON);
      trajectoryBack = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryBackJSON, ex.getStackTrace());
    }

    addCommands(
      new SequentialCommandGroup
      (

        new InstantCommand(driveTrain::zeroSensors),
        new InstantCommand(() -> new IntakeCmd(intake).runIntakeForward()),
        new InstantCommand(shooter::initiation),
        new InstantCommand(roller::initiation),

        new ParallelCommandGroup(
          new TaxiCmd(driveTrain), 
          new InstantCommand(intake::dropIntake) 
        ),
        new TurnToAngleCmd(driveTrain, () -> 0.0),
        new AutoFeedCmd(feeder, false),
        new InstantCommand(shooter::stopShooter),
        new InstantCommand(roller::stopRoller),


        // new ParallelCommandGroup
        // (

          
        //   new ShooterCmd(shooter, false, true),   
        //   new RollerCmd(roller, false, true),

        //   new SequentialCommandGroup
        //   (      
        //     new ParallelCommandGroup
        //     (
        //         new TaxiCmd(driveTrain),
        //         new InstantCommand(() -> new IntakeCmd(intake).runIntakeForward())
        //     ),
        //     new AutoFeedCmd(feeder, true)  
        //   )          
        // ),

        
        // new TurnToAngleCmd(driveTrain, () -> 0.0),

        new InstantCommand
        (
          driveTrain::setPose
        ),


        new RamseteCommand
        (
          trajectoryGetBall, 
          driveTrain::getPose, 
          new RamseteController(), 
          new SimpleMotorFeedforward(Constants.DriveConstants.kDriveKS, Constants.DriveConstants.kDriveKV, Constants.DriveConstants.kDriveKA), 
          new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters),
          driveTrain::getWheelSpeeds, 
          new PIDController(Constants.DriveConstants.kDrivekP, 0, 0),
          new PIDController(Constants.DriveConstants.kDrivekP, 0, 0), 
          driveTrain::tankDriveVolts,
          driveTrain
        ),

        

        new RamseteCommand
        (
          trajectoryBack, 
          driveTrain::getPose, 
          new RamseteController(), 
          new SimpleMotorFeedforward(Constants.DriveConstants.kDriveKS, Constants.DriveConstants.kDriveKV, Constants.DriveConstants.kDriveKA), 
          new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters),
          driveTrain::getWheelSpeeds, 
          new PIDController(Constants.DriveConstants.kDrivekP, 0, 0),
          new PIDController(Constants.DriveConstants.kDrivekP, 0, 0), 
          driveTrain::tankDriveVolts,
          driveTrain
        ),

        new InstantCommand(intake::stopSubsystemIntake),

        new TurnToAngleCmd(driveTrain, () -> 0.0),

        new ParallelRaceGroup(
          new InstantCommand(shooter::visionShooter),
          new InstantCommand(roller::visionRoller)


          // new AutoFeedCmd(feeder)
          
        )
        // new InstantCommand(feeder::stopSubsystemFeed)

      
      
      ));

      // new SequentialCommandGroup(
      // // );
        
      //   new ParallelCommandGroup(
        
      //   //new InstantCommand(() -> new SetDriveCmd(driveTrain).setDriveAuto(-0.3,0.0)),
      //   new InstantCommand(() -> new IntakeCmd(intake).runIntakeForward()),

      //   new AutoAlignCmd(driveTrain))
        
      //   ),
      //   new TurnToAngleCmd(driveTrain, () -> 0.0),

  
      //   new ParallelRaceGroup(
      //     new AutoFeedCmd(feeder),
      //     new ShooterCmd(shooter),
      //     new RollerCmd(roller)
      //   )

        

        //RamseteCommand

       
      // );
  }

}
 