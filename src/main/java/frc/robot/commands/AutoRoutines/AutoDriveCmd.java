package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;


public class AutoDriveCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //drivesubsystem declaration
  private final DriveSubsystem driveTrain;
  private final Joystick joystick = new Joystick(0);

  //supplier lambda's declaration
  Supplier<Double> speedFunction, turnFunction;

  //linear and angular speed
  double xSpeed;
  double rotSpeed;

 
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
      if(SmartDashboard.getNumber("area",0)<4500 && SmartDashboard.getNumber("area",0)!=0)
      {
      driveTrain.drive(-0.1, (SmartDashboard.getNumber("angle", 0))/3);
      System.out.println("Angle here: " + SmartDashboard.getNumber("angle", 0));
      }
      else
      {
        driveTrain.drive(0.0,0.0);

      }


    }

}
