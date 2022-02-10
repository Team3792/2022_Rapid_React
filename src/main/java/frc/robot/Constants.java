// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /* Shooter */

    //Shooter PID constants
    public static double shooterkP = 0.002;
    public static double shooterkI = 0;
    public static double shooterkD = 0;

    //FF constants
    public static double shooterKs = 0.0015;
    public static double shooterKv = 0.00325;
    public static double shooterKa = 0.003;

    /* Climb */

    //Climb PID constants
    public static double climbkP = 0;
    public static double climbkI = 0;
    public static double climbkD = 0;

     //Arm PID constants
     public static double armkP = 0;
     public static double armkI = 0;
     public static double armkD = 0;


}
