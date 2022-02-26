// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.AA3792.CustomButtons.CombinedButton;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.button.Button;

import com.AA3792.Constants;

import edu.wpi.first.wpilibj.XboxController;


/** An example command that uses an example subsystem. */
public class LTriggerSquare extends Button {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private XboxController stick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LTriggerSquare(XboxController stick) 
  {
      this.stick = stick;

    }


  public boolean get()
  {
    if ((stick.getRawAxis(Constants.ButtonConstant.kLTrigger) >= 0.7) && (stick.getRawButton(Constants.ButtonConstant.kSquareButton)))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

 




}