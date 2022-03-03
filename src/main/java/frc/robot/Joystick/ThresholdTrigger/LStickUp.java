// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Joystick.ThresholdTrigger;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class LStickUp extends Button {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private XboxController stick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LStickUp(XboxController stick) 
  {
      this.stick = stick;

    }



  public boolean get()
  {
    if (stick.getRawAxis(Constants.ButtonConstant.kLYAxis) >= 0.8)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

 




}