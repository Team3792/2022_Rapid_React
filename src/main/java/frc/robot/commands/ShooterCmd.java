package frc.robot.commands;

import frc.robot.subsystems.ShooterPID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.controller.PIDController;

public class ShooterCmd extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterPID shooter; 

  public ShooterCmd(ShooterPID s) {
       shooter = s;
    // Use addRequirements() here to declare subsystem dependencies.
       addRequirements(shooter);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new PIDCommand(
        new PIDController(
            Constants.shooterkP,
            Constants.shooterkI,
            Constants.shooterkD),
        // Close the loop on the turn rate
        shooter::getMeasurement,
        // Setpoint is in subsystem
        ShooterPID.setpoint(),
        // Pipe the output to the feedforward control
        output -> shooter.useOutput(output),
        // Require the shooter
        shooter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
