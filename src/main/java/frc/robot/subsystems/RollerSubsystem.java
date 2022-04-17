// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
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
    // /* Factory Default all hardware to prevent unexpected behaviour */
    // roller.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
		roller.configNeutralDeadband(0.001);

    /* Config current limiting */
    roller.enableCurrentLimit(true);
    roller.configPeakCurrentDuration(500, 30);
    roller.configPeakCurrentLimit(80, 30);

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
    roller.config_IntegralZone(Constants.RollerConstants.kPIDLoopIdx, Constants.RollerConstants.krollerIz, Constants.RollerConstants.kTimeoutMs);

    // roller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
  }

  public void setRoller(double rpm)
    {
      roller.set(ControlMode.Velocity, toRaw(rpm));
    }
    public void initiation()
    {
      setRoller(9652);
    }

    public void lowPort()
    {
      setRoller(-2300);
    }

    public void reverse()
    {
      setRoller(-3000);
    }

  // public void fiveKRPMidk()
  //   {
  //     // setRoller((((joystick.getRawAxis(3) + 1) / 4) * 7500) * 3.7);
  //     setRoller(SmartDashboard.getNumber("Roller", 3000));
  //   }

    public void visionRoller()
    {
      // System.out.println("Roller: " + getRollerRPM());
      setRoller(getRollerRPM());

    }
    
  public double getRollerRPM() {
    double xVal = SmartDashboard.getNumber("targetDist", 84);
    double kShootVal;
    if (xVal <= 93) {
        kShootVal = 0.0000414414259424234 * Math.pow(xVal, 5) 
                - 0.0149920706919886  * Math.pow(xVal, 4)  
                + 2.13797087428725  * Math.pow(xVal, 3)  
                - 150.151797625819  * Math.pow(xVal, 2)  
                + 5202.12230744896  * Math.pow(xVal, 1)  
                - 68255.4031903005;
    } else {
        kShootVal = 0.00105705403208844 * Math.pow(xVal, 4) 
                - 0.473451255106056 * Math.pow(xVal, 3) 
                + 78.9367234383612  * Math.pow(xVal, 2) 
                - 5795.36686845961  * Math.pow(xVal, 1) 
                + 160476.696335883;
    }
    double friction = 0.45;
    double OmegaT;
    double vVal;
    double OmegaB;
    double RPM; 
    xVal += 47;

    OmegaB = (kShootVal*(2*Math.PI)/60); 
    vVal = (xVal)/(0.3907311285*Math.sqrt((-81.5+2.36*xVal)/192)); 
    System.out.println("vVal: " + vVal); 
    OmegaT = -(vVal-2*OmegaB*2*Math.PI)/(friction*2*Math.PI); 
    RPM = (OmegaT*60)/(2*Math.PI); 
    if (xVal < 97){
      return -0.1*RPM;
    }
    else if (xVal<140) { 
      return 0.7*RPM; 
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

