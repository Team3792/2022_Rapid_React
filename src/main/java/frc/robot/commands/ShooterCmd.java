package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterCmd extends PIDCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterSubsystem shooter; 
    private double setpoint;

    //timer for auto init
    private final Timer timer;

    //bool for auto or not + cutoff
    private boolean autoStatus;
    private boolean complete;

  public ShooterCmd(ShooterSubsystem shooter, double setpoint, boolean autoStatus) {
      super(
      new PIDController(Constants.ShooterConstants.shooterkP, Constants.ShooterConstants.shooterkI, Constants.ShooterConstants.shooterkD),
       // Close the loop on the turn rate
       shooter::getMeasurement,
       // Setpoint is in subsystem
        setpoint,
       // Pipe the output to the feedforward control
       output -> shooter.useOutput(output, setpoint),
       // Require the shooter
        shooter
      // Use addRequirements() here to declare subsystem dependencies. 
      );
      this.shooter = shooter;
      this.setpoint = setpoint;
      addRequirements(shooter);
      
      timer = new Timer();
      timer.start();

      this.autoStatus = autoStatus;

      complete = false;

      SmartDashboard.putNumber("Setpoint", setpoint());
    }
 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
   shooter.zero();
 }

 
 public double setpoint() {
  double setpointVal = setpoint;
  return (setpointVal);
}
 
 @Override
 public void execute() {
   if(autoStatus && timer.hasElapsed(5.0)){
    complete = true;
    System.out.println("done");
   }
   else{
    super.execute();
    System.out.println("in progress");
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
