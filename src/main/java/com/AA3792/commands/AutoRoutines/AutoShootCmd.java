package com.AA3792.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.AA3792.commands.ShooterCmd;
import com.AA3792.subsystems.DriveSubsystem;
import com.AA3792.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;


public class AutoShootCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final ShooterSubsystem shooter;
  private final Timer timer;
  private final ShooterCmd shoot;
  private boolean complete = false;

  @Override
  public boolean isFinished() {
    return complete;
  }
 
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShootCmd(ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.timer = new Timer();
    shoot = new ShooterCmd(shooter, () -> 0.2);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    timer.start();

    
    }
    @Override
    public void execute() {
      
      if(!timer.hasElapsed(5.0))
      {
        shoot.execute();
        System.out.println("Moved to shoot");
      }else {
        complete = true;
      }

    }
}