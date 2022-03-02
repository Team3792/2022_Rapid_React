// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Joystick;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Joystick.ThresholdTrigger.*;
import frc.robot.commands.Joystick.CombinedButton.*;



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

    public final JoystickButton TriangleButton;
    public final JoystickButton SquareButton;
    public final JoystickButton XButton;
    public final JoystickButton CircleButton;

    public final JoystickButton L1Button;
    public final JoystickButton R1Button;


    public final JoystickButton LFaceButton;
    public final JoystickButton RFaceButton;

    public final JoystickButton LStickButton;
    public final JoystickButton RStickButton;


    public final LStickUp climbUp;
    public final LStickDown climbDown;
    public final RStickLeft pivotBack;
    public final RStickRight pivotForward;

    public final LTriggerCircle LTriggerCircleButton;
    public final LTriggerSquare LTriggerSquareButton;
    public final LTriggerTriangle LTriggerTriangleButton;
    public final LTriggerX LTriggerXButton;

    public final RTrigger RTriggerButton;


    

    
  public PS5Mapping() {

    POVUpish = new POVButton(operateController, 315).or(new POVButton(operateController, 0)).or(new POVButton(operateController, 45));
    POVDownish = new POVButton(operateController, 225).or(new POVButton(operateController, 180)).or(new POVButton(operateController, 135));
    POVLeftish = new POVButton(operateController, 225).or(new POVButton(operateController, 270)).or(new POVButton(operateController, 315));
    POVRightish = new POVButton(operateController, 45).or(new POVButton(operateController, 90)).or(new POVButton(operateController, 135));

    POVUp = new POVButton(operateController, 0);
    POVRight = new POVButton(operateController, 90);
    POVDown = new POVButton(operateController, 180);
    POVLeft = new POVButton(operateController, 270);

    XButton = new JoystickButton(operateController, Constants.ButtonConstant.kXButton);
    CircleButton = new JoystickButton(operateController, Constants.ButtonConstant.kCircleButton);
    SquareButton = new JoystickButton(operateController, Constants.ButtonConstant.kSquareButton);
    TriangleButton = new JoystickButton(operateController, Constants.ButtonConstant.kTriangleButton);


    L1Button = new JoystickButton(operateController, Constants.ButtonConstant.kL1Button);
    R1Button = new JoystickButton(operateController, Constants.ButtonConstant.kR1Button);



    climbUp = new LStickUp(operateController);
    climbDown = new LStickDown(operateController);
    pivotBack = new RStickLeft(operateController);
    pivotForward = new RStickRight(operateController);

    LTriggerCircleButton = new LTriggerCircle(operateController);
    LTriggerSquareButton = new LTriggerSquare(operateController);
    LTriggerTriangleButton = new LTriggerTriangle(operateController);
    LTriggerXButton = new LTriggerX(operateController);

    RTriggerButton = new RTrigger(operateController);




    LFaceButton = new JoystickButton(operateController, 7);
    RFaceButton = new JoystickButton(operateController, 8);

    LStickButton = new JoystickButton(operateController, 9);
    RStickButton = new JoystickButton(operateController, 10);



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