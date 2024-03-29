// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCmd(IntakeSubsystem intake) {
     this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  public void runIntakeForward(){
    intake.setValue(.8);
    SmartDashboard.putBoolean("Intake", true);
  }
  public void runIntakeForwardSlow(){
    intake.setValue(.4);
    SmartDashboard.putBoolean("Intake", true);

  }

  public void runIntakeBackward(){
    intake.setValue(-0.5);
    SmartDashboard.putBoolean("Intake", true);

  }

  public void stopIntake(){
    intake.setValue(0);
    SmartDashboard.putBoolean("Intake", false);

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