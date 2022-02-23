// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class FeedSubsystem extends SubsystemBase {
  private final WPI_VictorSPX feedMotor = new WPI_VictorSPX(Constants.MotorID.kFeedMotor);
  
  public FeedSubsystem() {}

  public void setValue(double v){
    feedMotor.set(v);
  }

}

