package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RollerSubsystem;


public class RollerCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RollerSubsystem roller; 
    private Supplier<Double> input;
    private boolean vision;
    private boolean preSpin;

    //timer for auto init
    private final Timer timer;

    //bool for auto or not + cutoff
    private boolean complete;
    Joystick driveJoystick = new Joystick(Constants.ButtonConstant.kDriveJoystick);

  public RollerCmd(RollerSubsystem roller, boolean vision, boolean preSpin) {
      this.roller = roller;
      this.vision = vision;
      this.preSpin = preSpin;
      System.out.println("Reached roller Cmd");

      timer = new Timer();
      timer.start();
    


      complete = false;
    }

  @Override
  public void initialize() {
    // roller.setRoller(input.get());
    // SmartDashboard.putNumber("input.get", input.get());
    // System.out.println("Input: " + input.get());
  }

  @Override
  public void execute() 
  {
   
    if (preSpin)
    {
     if(timer.hasElapsed(1.0))
     {
      roller.initiation();
     }
     if (timer.hasElapsed(6.0))
     {
      complete = true;
     }

       // System.out.println("in progress");
     
    }
 
    else if (!preSpin)
    {
     if(timer.hasElapsed(8.0)){
       complete = true;
       // System.out.println("done");
      }
      else
      {
       if (vision)
       {
         roller.visionRoller();
       }
       else if (!vision)
       {
         roller.initiation();
       }
       // System.out.println("in progress");
      }
    }
 }

 // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roller.stopRoller();

 }

  @Override
  public boolean isFinished() {
    return complete;
  } 

}
