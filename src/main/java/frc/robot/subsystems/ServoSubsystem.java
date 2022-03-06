// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.Servo;




import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ServoSubsystem extends SubsystemBase {
  
  Servo LArmServo = new Servo(1);
  Servo RArmServo = new Servo(0);

  
  public ServoSubsystem() 
  {
  }

  public void turnLServoAngle(double v)
  {
    LArmServo.setAngle(v);
  }

  public void turnRServoAngle(double v)
  {
    RArmServo.setAngle(v);
  }

}

