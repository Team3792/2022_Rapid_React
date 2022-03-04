// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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