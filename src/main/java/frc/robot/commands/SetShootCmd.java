// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class SetShootCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterSubsystem shooter;
    private final Timer timer;
    private boolean complete;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetShootCmd(ShooterSubsystem shooter) {
     this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

      timer = new Timer();
      timer.start();
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() >= 5.0){
      complete = true;
      shooter.zero();
    }
    else{
      shooter.setValue(0.3);
    }
  }

  public void reverseShooter()
  {
    shooter.setValue(-0.2);
  }

  public void stopShoot()
  {
    shooter.setValue(0);
  }
  
  public void forwardShoot()
  {
    shooter.setValue(0.3);
  }


 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return complete;
  }
}