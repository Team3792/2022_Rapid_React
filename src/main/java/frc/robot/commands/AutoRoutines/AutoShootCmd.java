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
  private final ShooterCmd shoot;
 
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


    
    }
    @Override
    public void execute() {
      timer.start();
      while(!timer.hasElapsed(1.0)){
        shoot.execute();
      }
      while(!timer.hasElapsed(2.0)){
        shoot.execute();
        
      }

    }
}
