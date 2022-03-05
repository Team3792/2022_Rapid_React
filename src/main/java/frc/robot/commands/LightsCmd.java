// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AAPowerDistribution;
import frc.robot.subsystems.LEDSubsystem;


/** An example command that uses an example subsystem. */
public class LightsCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private AAPowerDistribution aaPDH;
  private LEDSubsystem ledLights;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LightsCmd(AAPowerDistribution aaPDH) {
    this.aaPDH = aaPDH;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(aaPDH);
    }


  public LightsCmd(LEDSubsystem ledLights)
  {
    this.ledLights = ledLights;
    addRequirements(ledLights);
  }
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // public void ringLightOn()
  // {
  //   aaPDH.setRingLight(true);
  // }

  // public void ringLightOff()
  // {
  //   aaPDH.setRingLight(false);
  // }


  



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}