// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutoRoutines;

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
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;




/** default drive using the DriveSubystem. */
public class PathTest extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final DriveSubsystem driveTrain;
  private final IntakeSubsystem intake;
  private final FeedSubsystem feeder;
  private final ShooterSubsystem shooter;
  private final RollerSubsystem roller;

  //values for controlling stages
  boolean driveDone;
  boolean shootDone;
 
  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public PathTest(DriveSubsystem driveTrain, IntakeSubsystem intake, FeedSubsystem feeder, ShooterSubsystem shooter, RollerSubsystem roller) {
    this.driveTrain = driveTrain;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;
    this.roller = roller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    var autoVoltageConstraint = 
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.DriveConstants.kDriveKS,
        Constants.DriveConstants.kDriveKV,
        Constants.DriveConstants.kDriveKA),
      new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters),
      11);

    TrajectoryConfig config = new TrajectoryConfig(
      Constants.DriveConstants.kMaxFastDriveSpeed,
      Constants.DriveConstants.kMaxAcceleration)
      .setKinematics(new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters))
      .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(5, 1.8, new Rotation2d(214)),
        // Pass through these two interior waypoints, making a straight path
        List.of(new Translation2d(3.9, 1.5), new Translation2d(2.3, 1.25)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.05, 1.13, new Rotation2d(180)),
        // Pass config
        config);


    addCommands(


      new SequentialCommandGroup
      (
        new InstantCommand(driveTrain::zeroSensors),
        new InstantCommand(driveTrain::setPose),

        new RamseteCommand
        (
          exampleTrajectory, 
          driveTrain::getPose, 
          new RamseteController(), 
          new SimpleMotorFeedforward(Constants.DriveConstants.kDriveKS, Constants.DriveConstants.kDriveKV, Constants.DriveConstants.kDriveKA), 
          new DifferentialDriveKinematics(Constants.DriveConstants.kDriveTrainWidthMeters),
          driveTrain::getWheelSpeeds, 
          new PIDController(Constants.DriveConstants.kDrivekP, 0, 0),
          new PIDController(Constants.DriveConstants.kDrivekP, 0, 0), 
          driveTrain::tankDriveVolts,
          driveTrain
        )
       
    ));
  }

}
 