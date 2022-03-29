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

    //timer for auto init
    private final Timer timer;

    //bool for auto or not + cutoff
    private boolean autoStatus;
    private boolean complete;
    Joystick driveJoystick = new Joystick(Constants.ButtonConstant.kDriveJoystick);

  public RollerCmd(RollerSubsystem roller) {
      this.roller = roller;

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
  public void execute() {
   if(autoStatus && timer.hasElapsed(8.0)){
    complete = true;
    // System.out.println("done");
   }
   else{
    roller.visionRoller();
    // System.out.println("in progress");
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
