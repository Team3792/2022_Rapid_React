package frc.robot.commands.AutoRoutines;

import frc.robot.commands.ShooterCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;


public class AutoFeedCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final FeedSubsystem feeder;
  private final Timer timer;
 
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoFeedCmd(FeedSubsystem feeder) {
    this.feeder = feeder;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);


    
    }
    @Override
    public void execute() {
      timer.start();
      if(timer.hasElapsed(1.0)){
        feeder.setValue(0.8);
      }
      if(timer.hasElapsed(3.0)){
          feeder.setValue(0);
      }

    }
}
