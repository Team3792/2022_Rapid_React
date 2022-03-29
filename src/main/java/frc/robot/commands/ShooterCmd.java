package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public ShooterCmd(ShooterSubsystem shooter) {
      this.shooter = shooter;

      System.out.println("Reached Shooter Cmd");

      timer = new Timer();
      timer.start();
      



      complete = false;
    }



  @Override
  public void initialize() {
    // shooter.setShooter(input.get());
    // SmartDashboard.putNumber("input.get", input.get());
    // System.out.println("Input: " + input.get());


  }

  @Override
  public void execute() {
   if(timer.hasElapsed(8.0)){
    complete = true;
    // System.out.println("done");
   }
   else{
    shooter.visionShooter();


    // System.out.println("in progress");
   }
 }

 // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
 }

 public void runShooter(double rpm)
 {
   shooter.setShooter(rpm);
 }

  @Override
  public boolean isFinished() {
    return complete;
  } 

}
