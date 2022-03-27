// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class RollerSubsystem extends SubsystemBase {
  private final WPI_TalonSRX roller = new WPI_TalonSRX(Constants.MotorID.kRollerMotor);
  private Joystick joystick = new Joystick(0);


  @Override
    public void periodic() {
      SmartDashboard.putNumber("rollerRPM", getRPM());
      SmartDashboard.putNumber("rollerRPMGraph", getRPM());

    }
  
  public RollerSubsystem() 
  {
    // /* Factory Default all hardware to prevent unexpected behaviour */
    // roller.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
		roller.configNeutralDeadband(0.001);

    /* Config current limiting */
    roller.enableCurrentLimit(true);
    roller.configPeakCurrentDuration(1, 30);
    roller.configPeakCurrentLimit(100, 30);

		/* Config sensor used for Primary PID [Velocity] */
    roller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.RollerConstants.kPIDLoopIdx, Constants.RollerConstants.kTimeoutMs);

    /**
 *   Phase sensor accordingly. 
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
      // setRoller((((joystick.getRawAxis(3) + 1) / 4) * 7500) * 3.7);
      // setRoller(SmartDashboard.getNumber("BRUH", 3000)* SmartDashboard.getNumber("Ratio", 3.7));
      setRoller(SmartDashboard.getNumber("Roller", 1000));
    }
    
  public double getRollerRPM() {
    double currentDist = SmartDashboard.getNumber("targetDist", 0);
    double kShootVal = (2*Math.pow(10, -7))*Math.pow(currentDist, 6) - (Math.pow(10, -4))*Math.pow(currentDist, 5) + (0.0251 * Math.pow(currentDist, 4)) - (3.2537 * Math.pow(currentDist, 3)) + (229.32 * Math.pow(currentDist, 2)) - 8302.2 * currentDist + 123583;
    double friction = 0.45;
    double xVal = currentDist;
    double OmegaT;
    double vVal;
    double OmegaB;
    double RPM;

    OmegaB = (kShootVal*(2*Math.PI)/60); 
    vVal = (xVal)/(0.3907311285*Math.sqrt((-81.5+2.36*xVal)/192)); 
    System.out.println("vVal: " + vVal); 
    OmegaT = -(vVal-2*OmegaB*2*Math.PI)/(friction*2*Math.PI); 
    RPM = (OmegaT*60)/(2*Math.PI); 
    if (xVal<131) { 
      return 0.7*RPM; 
    } else if (xVal < 150){ 
      return 0.95*RPM; 
    } else{ 
      return 1.2*RPM; 
    }
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

