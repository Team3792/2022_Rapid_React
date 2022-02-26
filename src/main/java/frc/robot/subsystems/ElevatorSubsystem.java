// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;




import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorSubsystem extends SubsystemBase {
  private final WPI_TalonFX leftElevatorMotor = new WPI_TalonFX(Constants.MotorID.kLeftElevatorMotor);
  private final WPI_TalonFX rightElevatorMotor = new WPI_TalonFX(Constants.MotorID.kRightElevatorMotor);


  private final MotorControllerGroup elevatorMotors = new MotorControllerGroup(leftElevatorMotor, rightElevatorMotor);

  //public final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLead, leftFollow);


  
  public ElevatorSubsystem() {

    //rightElevatorMotor.follow(leftElevatorMotor);
    leftElevatorMotor.setInverted(true);
  }

  public void setValue(double v)
  {
    elevatorMotors.set(v);

  }

}

