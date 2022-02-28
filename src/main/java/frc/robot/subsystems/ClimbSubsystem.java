// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;




import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class ClimbSubsystem extends SubsystemBase {
  private final WPI_TalonFX leftArmMotor = new WPI_TalonFX(Constants.MotorID.kLeftArmMotor);
  private final WPI_TalonFX rightArmMotor = new WPI_TalonFX(Constants.MotorID.kRightArmMotor);


  private final MotorControllerGroup armMotors = new MotorControllerGroup(leftArmMotor, rightArmMotor);

  //public final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLead, leftFollow);


  
  public ClimbSubsystem() 
  {
    leftArmMotor.setInverted(true);
  }

  public void setValue(double v)
  {
    armMotors.set(v);

  }

}

