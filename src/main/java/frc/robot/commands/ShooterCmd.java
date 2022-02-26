package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;


public class ShooterCmd extends PIDCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterSubsystem shooter; 
    private Supplier<Double> inputFunction;

    //timer for auto init
    private final Timer timer;

    //bool for auto or not + cutoff
    private boolean autoStatus;
    private boolean complete;

  public ShooterCmd(ShooterSubsystem shooter, Supplier<Double> stickInput, boolean autoStatus) {
      super(
      new PIDController(Constants.ShooterConstants.shooterkP, Constants.ShooterConstants.shooterkI, Constants.ShooterConstants.shooterkD),
       // Close the loop on the turn rate
       shooter::getMeasurement,
       // Setpoint is in subsystem
        stickInput.get(),
       // Pipe the output to the feedforward control
       output -> shooter.useOutput(output, stickInput.get()),
       // Require the shooter
        shooter
      // Use addRequirements() here to declare subsystem dependencies. 
      );
      inputFunction = stickInput;
      this.shooter = shooter;
      addRequirements(shooter);
      
      timer = new Timer();
      timer.start();

      this.autoStatus = autoStatus;

      complete = false;
    }
 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
   shooter.zero(inputFunction.get());
 }

 
 @Override
 public void execute() {
   if(autoStatus && timer.hasElapsed(5.0)){
    complete = true;
   }
   else{
    super.execute();
   }
 }

 @Override
    public boolean isFinished() {
      return complete;
  } 
 /** no overrides needed of reg PID command 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  **/

}