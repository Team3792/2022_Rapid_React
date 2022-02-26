// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import frc.robot.Constants;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.DefaultDriveCmd;
import frc.robot.commands.AutoRoutines.*;
import frc.robot.commands.IntakeCmd;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Supplier;




/** default drive using the DriveSubystem. */
public class Auto2Ball extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final DriveSubsystem driveTrain;
  private final IntakeSubsystem intake;
  private final FeedSubsystem feeder;
  private final ShooterSubsystem shooter;

  //values for controlling stages
  boolean driveDone;
  boolean shootDone;
 
  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public Auto2Ball(DriveSubsystem driveTrain, IntakeSubsystem intake, FeedSubsystem feeder, ShooterSubsystem shooter) {
    this.driveTrain = driveTrain;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);


    addCommands(

      new AutoAlignCmd(driveTrain, shooter, feeder),

      new ParallelCommandGroup(
        new AutoShootCmd(shooter),

        new AutoFeedCmd(feeder)
      )


    );
    
  }

}
 