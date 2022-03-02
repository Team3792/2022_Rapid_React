// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.Servo;




import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ServoSubsystem extends SubsystemBase {
  
  Servo testServo = new Servo(0);
  //public final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLead, leftFollow);


  
  public ServoSubsystem() 
  {
  }

  public void setValue(double v)
  {
    testServo.set(v);
  }

}

