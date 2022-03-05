// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Joystick.PS5Mapping;

/** Add your docs here. */
public class ClimbSubsystem extends SubsystemBase {
  public final WPI_TalonFX leftClimbMotor = new WPI_TalonFX(Constants.MotorID.kLeftArmMotor);
  public final WPI_TalonFX rightClimbMotor = new WPI_TalonFX(Constants.MotorID.kRightArmMotor);

	public final PS5Mapping operateController = new PS5Mapping();
	public final XboxController controller = new XboxController(1);

	public TalonFXConfiguration leftConfig = new TalonFXConfiguration();
	public TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  int smoothing;
  
  public ClimbSubsystem() {		

  /* Disable all motors */
	rightClimbMotor.set(TalonFXControlMode.PercentOutput, 0);
	leftClimbMotor.set(TalonFXControlMode.PercentOutput,  0);
	
	/* Set neutral modes */
	leftClimbMotor.setNeutralMode(NeutralMode.Brake);
	rightClimbMotor.setNeutralMode(NeutralMode.Brake);

	/* Configure output */
	// rightClimbMotor.setInverted(TalonFXInvertType.CounterClockwise);
	// leftClimbMotor.setInverted(TalonFXInvertType.Clockwise);
	
	// TalonFXInvertType rightInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	// TalonFXInvertType leftInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	rightClimbMotor.setInverted(TalonFXInvertType.Clockwise);
	leftClimbMotor.setInverted(TalonFXInvertType.CounterClockwise);
	
	TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"
	TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	
	/** Feedback Sensor Configuration */

	/** Distance Configs */

	/* Configure Feedback Sensors */
	leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Feedback Source

	/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
	rightConfig.remoteFilter0.remoteSensorDeviceID = leftClimbMotor.getDeviceID(); //Device ID of Remote Source
	rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
	
	/* Now that the Left sensor can be used by the master Talon,
	 * set up the Left (Aux) and Right (Master) distance into a single
	 * Robot distance as the Master's Selected Sensor 0. */
	setRobotDistanceConfigs(rightInvert, rightConfig);

	/* Setup difference signal to be used to keep climbs in phase */
	setRobotTurnConfigs(rightInvert, rightConfig);

	/* FPID for Distance */
	rightConfig.slot0.kP = Constants.ClimbConstants.kClimbP;
	rightConfig.slot0.kI = Constants.ClimbConstants.kClimbI;
	rightConfig.slot0.kD = Constants.ClimbConstants.kClimbD;
	rightConfig.slot0.kF = Constants.ClimbConstants.kClimbF;
	rightConfig.slot0.integralZone = Constants.ClimbConstants.kClimbIzone;
	rightConfig.slot0.closedLoopPeakOutput = Constants.ClimbConstants.kClimbPeakOutput;
	rightConfig.slot0.allowableClosedloopError = 50;

	/* FPID for Correction */
	rightConfig.slot1.kP = Constants.ClimbAuxConstants.kClimbAuxP;
	rightConfig.slot1.kI = Constants.ClimbAuxConstants.kClimbAuxI;
	rightConfig.slot1.kD = Constants.ClimbAuxConstants.kClimbAuxD;
	rightConfig.slot1.kF = Constants.ClimbAuxConstants.kClimbAuxF;
	rightConfig.slot1.integralZone = Constants.ClimbAuxConstants.kClimbAuxIzone;
	rightConfig.slot1.closedLoopPeakOutput = Constants.ClimbAuxConstants.kClimbAuxPeakOutput;
	rightConfig.slot0.allowableClosedloopError = 0;

	/** Config the neutral deadband. */
	rightConfig.neutralDeadband = .001;
	leftConfig.neutralDeadband = .001;
	/**
	 * 1ms per loop.  PID loop can be slowed down if need be.
	 */
	int closedLoopTimeMs = 1;
	rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
	rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
	rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
	rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

	/* Motion Magic Configs */
	rightConfig.motionAcceleration = Constants.ClimbConstants.kClimbAccel; //(distance units per 100 ms) per second
	rightConfig.motionCruiseVelocity = Constants.ClimbConstants.kClimbMaxV; // distance units per 100 ms

	/* APPLY the config settings */
	leftClimbMotor.configAllSettings(leftConfig);
	rightClimbMotor.configAllSettings(rightConfig);

	/* Set status frame periods to ensure we don't have stale data */
	rightClimbMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 30);
	rightClimbMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 30);
	rightClimbMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, 30);
	rightClimbMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, 30);
	leftClimbMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);

	/* Initialize */
	rightClimbMotor.selectProfileSlot(0, 0);
	rightClimbMotor.selectProfileSlot(1, 1);
	rightClimbMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
	zeroSensors();
	}

	public void moveClimbMM(double Setpoint){
		rightClimbMotor.set(TalonFXControlMode.MotionMagic, Setpoint);
    // , DemandType.AuxPID, 0);
		leftClimbMotor.follow(rightClimbMotor);
    // , FollowerType.AuxOutput1);
  }

  public void moveClimbForward(){
		rightClimbMotor.set(TalonFXControlMode.PercentOutput, 0.05);
    // , DemandType.AuxPID, 0
		leftClimbMotor.follow(rightClimbMotor);
    // , FollowerType.AuxOutput1
	}

  public void moveClimbBack(){
		rightClimbMotor.set(TalonFXControlMode.PercentOutput, -0.05);
    // , DemandType.AuxPID, 0
		leftClimbMotor.follow(rightClimbMotor);
    // , FollowerType.AuxOutput1
	}

  public void moveLeftUp(){
		leftClimbMotor.set(TalonFXControlMode.PercentOutput, 0.05);
	}

	public void stopClimb(){
		rightClimbMotor.set(0);
		leftClimbMotor.set(0);
	}

  //can add something for smoothing: rightClimbMotor.gMotionSCurveStrength(smoothing);

	public void zeroSensors() {
    leftClimbMotor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    rightClimbMotor.getSensorCollection().setIntegratedSensorPosition(0, 30);
    System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
}

  public void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
    /**
     * Determine if we need a Sum or Difference.
     * 
     * The auxiliary Talon FX will always be positive
     * in the forward direction because it's a selected sensor
     * over the CAN bus.
     * 
     * The master's native integrated sensor may not always be positive when forward because
     * sensor phase is only applied to *Selected Sensors*, not native
     * sensor sources.  And we need the native to be combined with the 
     * aux (other side's) distance into a single robot distance.
     */

    /* THIS FUNCTION should not need to be modified. 
      This setup will work regardless of whether the master
      is on the Right or Left side since it only deals with
      distance magnitude.  */

    /* Check if we're inverted */
    if (masterInvertType == TalonFXInvertType.Clockwise){
      /* 
        If master is inverted, that means the integrated sensor
        will be negative in the forward direction.
        If master is inverted, the final sum/diff result will also be inverted.
        This is how Talon FX corrects the sensor phase when inverting 
        the motor direction.  This inversion applies to the *Selected Sensor*,
        not the native value.
        Will a sensor sum or difference give us a positive total magnitude?
        Remember the Master is one side of your drivetrain distance and 
        Auxiliary is the other side's distance.
          Phase | Term 0   |   Term 1  | Result
        Sum:  -((-)Master + (+)Aux   )| NOT OK, will cancel each other out
        Diff: -((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
        Diff: -((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
      */

      masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
      masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
      masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
    } else {
      /* Master is not inverted, both sides are positive so we can sum them. */
      masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
      masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
      masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
    }

    /* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
      the real-world value */
    masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
  }


  public void setRobotTurnConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
    /**
     * Determine if we need a Sum or Difference.
     * 
     * The auxiliary Talon FX will always be positive
     * in the forward direction because it's a selected sensor
     * over the CAN bus.
     * 
     * The master's native integrated sensor may not always be positive when forward because
     * sensor phase is only applied to *Selected Sensors*, not native
     * sensor sources.  And we need the native to be combined with the 
     * aux (other side's) distance into a single robot heading.
     */

    /* THIS FUNCTION should not need to be modified. 
    This setup will work regardless of whether the master
    is on the Right or Left side since it only deals with
    heading magnitude.  */

    /* Check if we're inverted */
    if (masterInvertType == TalonFXInvertType.CounterClockwise){
      /* 
        If master is inverted, that means the integrated sensor
        will be negative in the forward direction.
        If master is inverted, the final sum/diff result will also be inverted.
        This is how Talon FX corrects the sensor phase when inverting 
        the motor direction.  This inversion applies to the *Selected Sensor*,
        not the native value.
        Will a sensor sum or difference give us a positive heading?
        Remember the Master is one side of your drivetrain distance and 
        Auxiliary is the other side's distance.
          Phase | Term 0   |   Term 1  | Result
        Sum:  -((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
        Diff: -((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
        Diff: -((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
      */

      masterConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
      masterConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
      masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1

      /*
        PID Polarity
        With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
        This is important because if the PID polarity is incorrect, we will run away while trying to correct
        Will inverting the polarity give us a positive counterclockwise heading?
        If we're moving counterclockwise(+), and the master is on the right side and inverted,
        it will have a negative velocity and the auxiliary will have a negative velocity
        heading = right + left
        heading = (-) + (-)
        heading = (-)
        Let's assume a setpoint of 0 heading.
        This produces a positive error, in order to cancel the error, the right master needs to
        drive backwards. This means the PID polarity needs to be inverted to handle this
        
        Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
        it will have a positive velocity and the auxiliary will have a positive velocity.
        heading = right + left
        heading = (+) + (+)
        heading = (+)
        Let's assume a setpoint of 0 heading.
        This produces a negative error, in order to cancel the error, the left master needs to
        drive forwards. This means the PID polarity needs to be inverted to handle this
      */
      masterConfig.auxPIDPolarity = true;
    } else {
      /* Master is not inverted, both sides are positive so we can diff them. */
      masterConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
      masterConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
      masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
      /* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
      masterConfig.auxPIDPolarity = true;
    }
  }

  @Override
	public void periodic() {
		SmartDashboard.putNumber("Right Climb Pos", rightClimbMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("Left Climb Pos", leftClimbMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("Right Climb V", (rightClimbMotor.getSelectedSensorVelocity()));
		SmartDashboard.putNumber("Left Climb V", (leftClimbMotor.getSelectedSensorVelocity()));
		SmartDashboard.putNumber("Right Climb Error", (rightClimbMotor.getSelectedSensorPosition()));
		SmartDashboard.putNumber("Left Climb Error", (leftClimbMotor.getSelectedSensorPosition()));
	}

}

