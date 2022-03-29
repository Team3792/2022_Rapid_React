package frc.robot.commands.Autonomous.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeedSubsystem;


public class AutoFeedCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final FeedSubsystem feeder;
  private final Timer timer;

  private boolean complete;
 
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

    complete = false;


    
    }
    @Override
    public void execute() {
      timer.start();
      if(timer.hasElapsed(1.5)){
        feeder.setValue(0.8);

      }
      if(timer.hasElapsed(5.0)){
          feeder.setValue(0);
          complete = true;
      }

    }

    @Override
    public boolean isFinished() {
      return complete;
  } 
}
