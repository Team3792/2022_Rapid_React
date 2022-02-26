// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.AA3792.CustomButtons;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/** An example command that uses an example subsystem. */
public class PS5Mapping extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private XboxController operateController = new XboxController(1);

    public final Trigger POVUpish;
    public final Trigger POVDownish;
    public final Trigger POVLeftish;
    public final Trigger POVRightish;

    public final Trigger POVUp;
    public final Trigger POVDown;
    public final Trigger POVLeft;
    public final Trigger POVRight;

    public final JoystickButton XButton;
    public final JoystickButton YButton;
    public final JoystickButton AButton;
    public final JoystickButton BButton;

    public final JoystickButton BumperL;
    public final JoystickButton BumperR;

    public final JoystickButton BackButton;
    public final JoystickButton StartButton;

    public final JoystickButton LeftStickButton;
    public final JoystickButton RightStickButton;


  public PS5Mapping() {

    POVUpish = new POVButton(operateController, 315).or(new POVButton(operateController, 0)).or(new POVButton(operateController, 45));
    POVDownish = new POVButton(operateController, 225).or(new POVButton(operateController, 180)).or(new POVButton(operateController, 135));
    POVLeftish = new POVButton(operateController, 225).or(new POVButton(operateController, 270)).or(new POVButton(operateController, 315));
    POVRightish = new POVButton(operateController, 45).or(new POVButton(operateController, 90)).or(new POVButton(operateController, 135));

    POVUp = new POVButton(operateController, 0);
    POVRight = new POVButton(operateController, 90);
    POVDown = new POVButton(operateController, 180);
    POVLeft = new POVButton(operateController, 270);

    XButton = new JoystickButton(operateController, XboxController.Button.kX.value);
    YButton = new JoystickButton(operateController, XboxController.Button.kY.value);
    AButton = new JoystickButton(operateController, XboxController.Button.kA.value);
    BButton = new JoystickButton(operateController, XboxController.Button.kB.value);

    BumperL = new JoystickButton(operateController, XboxController.Button.kLeftBumper.value);
    BumperR = new JoystickButton(operateController, XboxController.Button.kRightBumper.value);

    BackButton = new JoystickButton(operateController, XboxController.Button.kBack.value);
    StartButton = new JoystickButton(operateController, XboxController.Button.kStart.value);

    LeftStickButton = new JoystickButton(operateController, XboxController.Button.kLeftStick.value);
    RightStickButton = new JoystickButton(operateController, XboxController.Button.kRightStick.value);
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


}