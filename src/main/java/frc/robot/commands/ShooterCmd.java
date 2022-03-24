package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterSubsystem shooter; 
    private Supplier<Double> input;

    //timer for auto init
    private final Timer timer;

    //bool for auto or not + cutoff
    private boolean autoStatus;
    private boolean complete;
    Joystick driveJoystick = new Joystick(Constants.ButtonConstant.kDriveJoystick);

  public ShooterCmd(ShooterSubsystem shooter, Supplier<Double> input, boolean autoStatus) {
      this.shooter = shooter;

      timer = new Timer();
      timer.start();
      
      this.autoStatus = autoStatus;

      complete = false;
    }

  @Override
  public void initialize() {
    shooter.setShooter(input.get());
  }

  @Override
  public void execute() {
   if(autoStatus && timer.hasElapsed(8.0)){
    complete = true;
    // System.out.println("done");
   }
   else{

    // System.out.println("in progress");
   }
 }

 // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
 }

  @Override
  public boolean isFinished() {
    return complete;
  } 

}
