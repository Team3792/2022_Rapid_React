// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class RollerSubsystem extends SubsystemBase {
  
  private Encoder rollerEncoder = new Encoder(8,9);
  private final WPI_TalonSRX roller = new WPI_TalonSRX(Constants.MotorID.kRollerMotor);



  @Override
    public void periodic() {

        SmartDashboard.putNumber("distance", rollerEncoder.getDistance());

    SmartDashboard.putNumber("rate", rollerEncoder.getRate() * 60);
    }
  
  public RollerSubsystem() 
  {

    rollerEncoder.setDistancePerPulse(1./2048);

  }

  public void setRoller()
    {
        roller.set(0.45);
    }

    public void stopRoller()
    {
        roller.set(0);
    }

}

