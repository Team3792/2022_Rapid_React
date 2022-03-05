// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ServoSubsystem;

/** An example command that uses an example subsystem. */
public class ServoCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ServoSubsystem servo;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ServoCmd(ServoSubsystem servo) {
     this.servo = servo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(servo);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  public void openLServo()
  {
    servo.turnAngle(90);;
    // SmartDashboard.putBoolean("Servo", true);
  }

  
  

  public void resetServo()
  {
    servo.turnAngle(-10);
    // SmartDashboard.putBoolean("Servo", false);

  }

  public void openServo()
  {
    servo.turnAngle(10);;
    // SmartDashboard.putBoolean("Servo", true);
  }

  
  

  public void resetLServo()
  {
    servo.turnAngle(-10);
    // SmartDashboard.putBoolean("Servo", false);

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}