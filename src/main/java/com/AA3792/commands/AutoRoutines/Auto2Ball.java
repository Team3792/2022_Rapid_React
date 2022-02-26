// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.AA3792.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Supplier;

import com.AA3792.Constants;
import com.AA3792.commands.ClimbCmd;
import com.AA3792.commands.DefaultDriveCmd;
import com.AA3792.commands.IntakeCmd;
import com.AA3792.commands.AutoRoutines.*;
import com.AA3792.subsystems.ClimbSubsystem;
import com.AA3792.subsystems.DriveSubsystem;
import com.AA3792.subsystems.FeedSubsystem;
import com.AA3792.subsystems.IntakeSubsystem;
import com.AA3792.subsystems.ShooterSubsystem;




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
 