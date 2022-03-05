package frc.robot.commands.AutoRoutines;

import frc.robot.commands.ShooterCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
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
  private boolean complete = false;

  //output PID stuff
  private double output;
  private ShooterCmd shootCmd;

 
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShootCmd(ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    shootCmd = new ShooterCmd(shooter, ()-> 0.1, true);

    timer.start();
    
    }
    @Override
    public void execute() {
      
      if(!timer.hasElapsed(7.0))
      {
        output = shooter.getMeasurement();
        shooter.useOutput(output , 0.1);
      }
      else{
        shooter.zero(output);
        complete = true;
      }

    }


  @Override
  public boolean isFinished() {
    return complete;
  }
}
