// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutoRoutines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class AutoVision2Ball extends SequentialCommandGroup {
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
  public AutoVision2Ball(DriveSubsystem driveTrain, IntakeSubsystem intake, FeedSubsystem feeder, ShooterSubsystem shooter, RollerSubsystem roller) {
    this.driveTrain = driveTrain;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;
    this.roller = roller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);


    addCommands(

      new SequentialCommandGroup(
        
        new ParallelCommandGroup(
        

        //new InstantCommand(() -> new SetDriveCmd(driveTrain).setDriveAuto(-0.3,0.0)),
        new InstantCommand(() -> new IntakeCmd(intake).runIntakeForward()),

        new TaxiCmd(driveTrain))  
        ),
  
        new ParallelCommandGroup(
          new AutoFeedCmd(feeder, false),
          new ShooterCmd(shooter, true, false),
          new RollerCmd(roller, true, false)),

          new AutoDriveBackMore(driveTrain)


       
      );
  }

}
 