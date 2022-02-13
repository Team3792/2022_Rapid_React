// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Feeder extends SubsystemBase {
  private final WPI_TalonFX feeder = new WPI_TalonFX(Constants.MotorID.kFeedMotor);
  
  public Feeder() {}

  public void setValue(double v){
    feeder.set(v);
  }

}

