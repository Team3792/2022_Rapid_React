// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class RollerSubsystem extends SubsystemBase {
  private final WPI_TalonSRX roller = new WPI_TalonSRX(Constants.MotorID.kRollerMotor);

  @Override
    public void periodic() {
      SmartDashboard.putNumber("rollerRPM", getRPM());
    }
  
  public RollerSubsystem() 
  {
    /* Factory Default all hardware to prevent unexpected behaviour */
    roller.configFactoryDefault();

		/* Config sensor used for Primary PID [Velocity] */
        roller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                            Constants.RollerConstants.kPIDLoopIdx, 
                                            Constants.RollerConstants.kTimeoutMs);

        /**
		 * Phase sensor accordingly. 
         * Positive Sensor Reading should match Green (blinking) Leds on Talon
         */
		roller.setSensorPhase(true);

		/* Config the peak and nominal outputs */
		roller.configNominalOutputForward(0, Constants.RollerConstants.kTimeoutMs);
		roller.configNominalOutputReverse(0, Constants.RollerConstants.kTimeoutMs);
		roller.configPeakOutputForward(1, Constants.RollerConstants.kTimeoutMs);
		roller.configPeakOutputReverse(-1, Constants.RollerConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		roller.config_kF(Constants.RollerConstants.kPIDLoopIdx, Constants.RollerConstants.krollerF, Constants.RollerConstants.kTimeoutMs);
		roller.config_kP(Constants.RollerConstants.kPIDLoopIdx, Constants.RollerConstants.krollerP, Constants.RollerConstants.kTimeoutMs);
		roller.config_kI(Constants.RollerConstants.kPIDLoopIdx, Constants.RollerConstants.krollerI, Constants.RollerConstants.kTimeoutMs);
		roller.config_kD(Constants.RollerConstants.kPIDLoopIdx, Constants.RollerConstants.krollerD, Constants.RollerConstants.kTimeoutMs);
  }

  public void setRoller(double rpm)
    {
      roller.set(ControlMode.Velocity, toRaw(rpm));
    }

  public void fiveKRPMidk()
    {
      setRoller(5000);
    }
    
  public void stopRoller()
    {
      roller.set(0);
    }

  public double getRPM() 
    {
    return (toRPM(roller.getSelectedSensorVelocity()));
    }

  public double toRPM(double rawV) 
    {
      return (rawV * 600 / 4096);
    }

  public double toRaw(double rpm) 
    {
      return (rpm * 4096 / 600);
    }
}

