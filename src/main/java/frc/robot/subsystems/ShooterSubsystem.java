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
        setShooter(2500);
    }

    public void initiation() {
        // setShooter((((joystick.getRawAxis(3) + 1) / 4) * 7500));
        setShooter(SmartDashboard.getNumber("BRUH", 3000));
    }

    public void leBron() {
        System.out.println(getShooterRPM());
        setShooter(getShooterRPM());
    }

    public double getShooterRPM() {
        double currentDist = 100;
        return (2*Math.pow(10, -7))*Math.pow(currentDist, 6) - (Math.pow(10, -4))*Math.pow(currentDist, 5) + (0.0251 * Math.pow(currentDist, 4)) - (3.2537 * Math.pow(currentDist, 3)) + (229.32 * Math.pow(currentDist, 2)) - 8302.2 * currentDist + 123583;
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