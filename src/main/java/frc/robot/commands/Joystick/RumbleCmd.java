// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Joystick;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


/** An example command that uses an example subsystem. */
public class RumbleCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private XboxController operateController = new XboxController(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RumbleCmd() {

    // Use addRequirements() here to declare subsystem dependencies.

    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }


  public void startShake()
  {
    operateController.setRumble(RumbleType.kLeftRumble, 0.5);
    operateController.setRumble(RumbleType.kRightRumble, 0.5);


  }

  public void stopShake()
  {
    operateController.setRumble(RumbleType.kLeftRumble, 0);
    operateController.setRumble(RumbleType.kRightRumble, 0);
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