// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.AA3792.CustomButtons;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


/** An example command that uses an example subsystem. */
public class RStickRight extends Button {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private XboxController operateController = new XboxController(1);
    private XboxController stick;
    private int axisNum;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RStickRight(XboxController stick) 
  {
      this.stick = stick;

    }



  public boolean get()
  {
    if (stick.getRawAxis(4) <= -0.8)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

 




}