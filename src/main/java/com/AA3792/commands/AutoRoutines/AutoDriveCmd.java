package com.AA3792.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.AA3792.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;


public class AutoDriveCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final DriveSubsystem driveTrain;

 
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveCmd(DriveSubsystem subsystem) {
    driveTrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);


    
    }
    @Override
    public void execute() {

      // SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
      while(SmartDashboard.getNumber("ballArea",0)<4500 && SmartDashboard.getNumber("ballArea",0)!=0)
      {
        driveTrain.drive(0.1, (SmartDashboard.getNumber("ballAngle", 0))/3);
        System.out.println("Angle here: " + SmartDashboard.getNumber("ballAngle", 0));   

    }
    driveTrain.drive(0.0,0.0);

}
}
