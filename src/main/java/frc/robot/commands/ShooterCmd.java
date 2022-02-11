package frc.robot.commands;

import frc.robot.subsystems.ShooterPID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.controller.PIDController;
import java.util.function.Supplier;

public class ShooterCmd extends PIDCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterPID shooter; 
    private Supplier<Double> inputFunction;

  public ShooterCmd(ShooterPID s, Supplier<Double> stickInput) {
      super(
      new PIDController(Constants.shooterkP, Constants.shooterkI, Constants.shooterkD),
       // Close the loop on the turn rate
       s::getMeasurement,
       // Setpoint is in subsystem
        stickInput.get(),
       // Pipe the output to the feedforward control
       output -> s.useOutput(output, stickInput.get()),
       // Require the shooter
        s
      // Use addRequirements() here to declare subsystem dependencies. 
      );
      inputFunction = stickInput;
      shooter = s;
      addRequirements(shooter);
    }
 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
   shooter.zero(inputFunction.get());
 }

 /** no overrides needed of reg PID command 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  **/

}
