// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCmd(ElevatorSubsystem elevator) {
     this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    /* Disable all motors */
		elevator.rightElevatorMotor.set(TalonFXControlMode.PercentOutput, 0);
		elevator.leftElevatorMotor.set(TalonFXControlMode.PercentOutput,  0);
		
		/* Set neutral modes */
		elevator.leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
		elevator.rightElevatorMotor.setNeutralMode(NeutralMode.Brake);

		/* Configure output */
		elevator.rightElevatorMotor.setInverted(true);
		
		TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	  	TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"
		
		/** Feedback Sensor Configuration */

		/** Distance Configs */

		/* Configure Feedback Sensors */
		elevator.leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Feedback Source

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		elevator.rightConfig.remoteFilter0.remoteSensorDeviceID = elevator.leftElevatorMotor.getDeviceID(); //Device ID of Remote Source
		elevator.rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
		
		/* Now that the Left sensor can be used by the master Talon,
		 * set up the Left (Aux) and Right (Master) distance into a single
		 * Robot distance as the Master's Selected Sensor 0. */
		elevator.setRobotDistanceConfigs(rightInvert, elevator.rightConfig);

		/* Setup difference signal to be used to keep elevators in phase */
		elevator.setRobotTurnConfigs(rightInvert, elevator.rightConfig);

		/* FPID for Distance */
		elevator.rightConfig.slot0.kP = Constants.ElevatorConstants.kElevatorP;
		elevator.rightConfig.slot0.kI = Constants.ElevatorConstants.kElevatorI;
		elevator.rightConfig.slot0.kD = Constants.ElevatorConstants.kElevatorD;
		elevator.rightConfig.slot0.kF = Constants.ElevatorConstants.kElevatorF;
		elevator.rightConfig.slot0.integralZone = Constants.ElevatorConstants.kElevatorIzone;
		elevator.rightConfig.slot0.closedLoopPeakOutput = Constants.ElevatorConstants.kElevatorPeakOutput;
		elevator.rightConfig.slot0.allowableClosedloopError = 0;

		/* false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * 
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		elevator.rightConfig.auxPIDPolarity = false;

		/* FPID for Correction */
		elevator.rightConfig.slot1.kP = Constants.ElevatorAuxConstants.kElevatorAuxP;
		elevator.rightConfig.slot1.kI = Constants.ElevatorAuxConstants.kElevatorAuxI;
		elevator.rightConfig.slot1.kD = Constants.ElevatorAuxConstants.kElevatorAuxD;
		elevator.rightConfig.slot1.kF = Constants.ElevatorAuxConstants.kElevatorAuxF;
		elevator.rightConfig.slot1.integralZone = Constants.ElevatorAuxConstants.kElevatorAuxIzone;
		elevator.rightConfig.slot1.closedLoopPeakOutput = Constants.ElevatorAuxConstants.kElevatorAuxPeakOutput;
		elevator.rightConfig.slot0.allowableClosedloopError = 0;

		/** Config the neutral deadband. */
		elevator.rightConfig.neutralDeadband = .001;
		elevator.leftConfig.neutralDeadband = .001;
		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 */
		int closedLoopTimeMs = 1;
		elevator.rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		elevator.rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		elevator.rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		elevator.rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

		/* Motion Magic Configs */
		elevator.rightConfig.motionAcceleration = 3301; //(distance units per 100 ms) per second
		elevator.rightConfig.motionCruiseVelocity = 16504; // distance units per 100 ms

		/* APPLY the config settings */
		elevator.leftElevatorMotor.configAllSettings(elevator.leftConfig);
		elevator.rightElevatorMotor.configAllSettings(elevator.rightConfig);

		/* Set status frame periods to ensure we don't have stale data */
		elevator.rightElevatorMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 30);
		elevator.rightElevatorMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 30);
		elevator.rightElevatorMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, 30);
		elevator.rightElevatorMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, 30);
		elevator.leftElevatorMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);

		/* Initialize */
		elevator.rightElevatorMotor.selectProfileSlot(0, 0);
		elevator.rightElevatorMotor.selectProfileSlot(1, 1);
		elevator.rightElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
		elevator.zeroSensors();
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

  public void moveElevatorUp(){
    elevator.rightElevatorMotor.set(TalonFXControlMode.MotionMagic, Constants.ElevatorConstants.setpoint, DemandType.AuxPID, 0);
    elevator.leftElevatorMotor.follow(elevator.rightElevatorMotor, FollowerType.AuxOutput1);
  }

  public void moveElevatorDown(){
    elevator.rightElevatorMotor.set(TalonFXControlMode.MotionMagic, 0, DemandType.AuxPID, 0);
    elevator.leftElevatorMotor.follow(elevator.rightElevatorMotor, FollowerType.AuxOutput1);
  }

  public void stopElevator(){
	  elevator.rightElevatorMotor.set(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (_withinThresholdLoops > kLoopsToSettle);
  }
}