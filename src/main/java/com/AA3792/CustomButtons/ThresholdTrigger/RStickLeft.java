// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.AA3792.CustomButtons.ThresholdTrigger;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.XboxController;
import com.AA3792.Constants;


/** An example command that uses an example subsystem. */
public class RStickLeft extends Button {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private XboxController stick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RStickLeft(XboxController stick) 
  {
      this.stick = stick;

    }



  public boolean get()
  {
    if (stick.getRawAxis(Constants.ButtonConstant.kRXAxis) >= 0.8)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

 




}