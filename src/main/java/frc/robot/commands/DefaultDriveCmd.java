// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;




/** default drive using the DriveSubystem. */
public class DefaultDriveCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final DriveSubsystem driveTrain;

  //supplier lambda's declaration
  Supplier<Double> speedFunction, turnFunction;

  //linear and angular speed
  double xSpeed;
  double rotSpeed;
 
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultDriveCmd(DriveSubsystem subsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    driveTrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    //double suppliers init
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //regular drive with inputted joystick values
    driveTrain.drive(speedFunction.get(), turnFunction.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
