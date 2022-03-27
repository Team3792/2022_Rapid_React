// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/** Add your docs here. */
public class AAPowerDistribution extends SubsystemBase {

  static PowerDistribution aaPDH = new PowerDistribution(1, ModuleType.kRev);

  public AAPowerDistribution() {}

  public static void ringLightOn()
  {
    aaPDH.setSwitchableChannel(true);
    System.out.println("turning on");
    Constants.GlobalStateConstants.kRingLightState = true;
    SmartDashboard.putBoolean("RingLED", true);

  }



  

  public static void ringLightOff()
  {
    aaPDH.setSwitchableChannel(true); //this is scuffed
    System.out.println("turning off");
    Constants.GlobalStateConstants.kRingLightState = false;
    SmartDashboard.putBoolean("RingLED", false);
  }

  

  public static void toggleLight()
  {
    if (!Constants.GlobalStateConstants.kRingLightState)
    {
      aaPDH.setSwitchableChannel(true);
      Constants.GlobalStateConstants.kRingLightState = true;
      SmartDashboard.putBoolean("RingLED", true);

    }
    else if (Constants.GlobalStateConstants.kRingLightState)
    {
      aaPDH.setSwitchableChannel(true); //this is scuffed
      Constants.GlobalStateConstants.kRingLightState = false;
      SmartDashboard.putBoolean("RingLED", false);
    }
  }

}

