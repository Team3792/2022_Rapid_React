// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonFX shooter = new WPI_TalonFX(Constants.MotorID.kShootMotor);
    private Joystick joystick = new Joystick(0);

    public ShooterSubsystem() {
        shooter.setInverted(false);
        shooter.setSelectedSensorPosition(0);
        
        // /* Factory Default all hardware to prevent unexpected behaviour */
		// shooter.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		shooter.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.ShooterConstants.kPIDLoopIdx, 
											Constants.ShooterConstants.kTimeoutMs);
											

		/* Config the peak and nominal outputs */
		shooter.configNominalOutputForward(0, Constants.ShooterConstants.kTimeoutMs);
		shooter.configNominalOutputReverse(0, Constants.ShooterConstants.kTimeoutMs);
		shooter.configPeakOutputForward(1, Constants.ShooterConstants.kTimeoutMs);
		shooter.configPeakOutputReverse(-1, Constants.ShooterConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		shooter.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kshooterF, Constants.ShooterConstants.kTimeoutMs);
		shooter.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kshooterP, Constants.ShooterConstants.kTimeoutMs);
		shooter.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kshooterI, Constants.ShooterConstants.kTimeoutMs);
		shooter.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kshooterD, Constants.ShooterConstants.kTimeoutMs);
        shooter.config_IntegralZone(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kshooterIz, Constants.ShooterConstants.kTimeoutMs);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // shooter.setSensorPhase(true);
    }

 

    @Override
    public void periodic() {
        SmartDashboard.putNumber("RPM", getRPM());
        SmartDashboard.putNumber("RPMGraph", getRPM());

        SmartDashboard.putNumber("targetRPM", SmartDashboard.getNumber("targetRPM", 0));
    }

    public void setShooter(double rpm){
        shooter.set(ControlMode.Velocity, toRaw(rpm));
    }

    public void lowPort() {
        setShooter(2300);
    }

    public void initiation() {
        // setShooter((((joystick.getRawAxis(3) + 1) / 4) * 7500));
        setShooter(SmartDashboard.getNumber("BRUH", 3000));

        System.out.println("ShooterLMAO " + getShooterRPM());
    }

    public void visionShooter()
    {
        setShooter(getShooterRPM());
    }

    public void leBron() {
        setShooter(3336);
    }

    public double getShooterRPM() {
        double xVal = SmartDashboard.getNumber("targetDist", 50);
        System.out.println("XVAL " + xVal);
        if (xVal <= 50)
        {
            return -0.0069879149* Math.pow(xVal, 3) 
                    + 0.9253397123 * Math.pow(xVal, 2) 
                    - 36.6293462365 * xVal 
                    + 5393.3848674349;
        }
        else if (xVal > 50 && xVal <= 93) {
            return 0.0000414414259424234 * Math.pow(xVal, 5)
                    - 0.0149920706919886  * Math.pow(xVal, 4)
                    + 2.13797087428725  * Math.pow(xVal, 3)
                    - 150.151797625819  * Math.pow(xVal, 2)
                    + 5202.12230744896  * Math.pow(xVal, 1)
                    - 68255.4031903005;
        } else {
            return 0.00105705403208844 * Math.pow(xVal, 4)
                    - 0.473451255106056 * Math.pow(xVal, 3)
                    + 78.9367234383612  * Math.pow(xVal, 2)
                    - 5795.36686845961  * Math.pow(xVal, 1)
                    + 160476.696335883;
        }
    }

    public void reverse() {
        setShooter(-1000);
    }

    public void stopShooter() {
        shooter.set(0);
    }

    public double getRPM() {
        double rpmVal = toRPM(shooter.getSelectedSensorVelocity());
        return (rpmVal);
    }

    private double toRPM(double rawV) {
        return (rawV * 600 * 2 / 2048);
    }

    private double toRaw(double rpm) {
        return (rpm * 2048.0 / 600.0 / 2);
    }

   

    
}