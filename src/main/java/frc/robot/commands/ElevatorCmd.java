// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ElevatorSubsystem elevator;

	public int kErrThreshold = 10; // how many sensor units until its close-enough
	public int kLoopsToSettle = 10; // how many loops sensor must be close-enough
	public int _withinThresholdLoops = 0;

  private Double setpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCmd(ElevatorSubsystem elevator, Double setpoint) {
     this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.setpoint = setpoint;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	  elevator.moveElevatorMM(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

	/* Check if closed loop error is within the threshld */
	if (elevator.rightElevatorMotor.getActiveTrajectoryPosition() < +kErrThreshold &&
	elevator.rightElevatorMotor.getActiveTrajectoryPosition() > -kErrThreshold) {

		++_withinThresholdLoops;
	} else {
		_withinThresholdLoops = 0;
	}
}

  public void stopElevator(){
	  elevator.rightElevatorMotor.set(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
	return (_withinThresholdLoops > kLoopsToSettle);
  }
}