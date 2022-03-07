// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonFX shooter = new WPI_TalonFX(Constants.MotorID.kShootMotor);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(Constants.ShooterConstants.shooterKs, Constants.ShooterConstants.shooterKv, Constants.ShooterConstants.shooterKa);
    


    public ShooterSubsystem() {
        shooter.setInverted(false);
        shooter.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("RPM", toRPM(getMeasurement()));
        SmartDashboard.putNumber("Error", SmartDashboard.getNumber("setpoint", 0.0) - toRPM(getMeasurement()));
    }

    public double setpoint() {
        double setpointVal = 5000;
        return (setpointVal);
    }

    public void zero() {
        shooter.set(0);
    }

    public void setValue(double v){
        shooter.set(v);
      }
    public void useOutput(double output, double setpoint) {
        System.out.println("setpoint: " + setpoint);
        shooter.setVoltage(output + shooterFF.calculate(setpoint));
    }

    public double getMeasurement() {
        double rpmVal = toRPM(shooter.getSelectedSensorVelocity());
        return (rpmVal);
    }

    public double toRPM(double rawV) {
        return (rawV * 60 * 10 * 2) / 2048;
    }
    
}