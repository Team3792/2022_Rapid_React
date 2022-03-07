// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class AAPowerDistribution extends SubsystemBase {

  static PowerDistribution aaPDH = new PowerDistribution(1, ModuleType.kRev);

  public AAPowerDistribution() {}

  public static void ringLightOn()
  {
    aaPDH.setSwitchableChannel(false);
  }

  

  public static void ringLightOff()
  {
    aaPDH.setSwitchableChannel(false);
  }

  // public static void toggleLight()
  // {
  //   if (Constants.GlobalStateConstants.kRingLightState == false)
  //   {
  //     aaPDH.setSwitchableChannel(true);
  //     Constants.GlobalStateConstants.kRingLightState = true;

  //   }
  //   else if (Constants.GlobalStateConstants.kRingLightState == true)
  //   {
  //     aaPDH.setSwitchableChannel(false);
  //     Constants.GlobalStateConstants.kRingLightState = false;
  //   }
  // }

}

