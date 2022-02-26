package frc.robot.commands.AutoRoutines;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;


public class AutoAlignCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //subsystem declarations
  private final DriveSubsystem driveTrain;
  private boolean complete = false;

  @Override
  public boolean isFinished() {
    return complete;
  } 
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlignCmd(DriveSubsystem m_drive) {
    driveTrain = m_drive;

     
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);


    
    }


    @Override
    public void execute() {
      if(Math.abs(SmartDashboard.getNumber("targetAngle", 0)) > 0.03){
        //SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
        driveTrain.drive(0, (SmartDashboard.getNumber("targetAngle", 0))/1);
        System.out.println("Angle here: " + SmartDashboard.getNumber("targetAngle", 0));  
      }
      else
      {
        Constants.stateCounter = 1;
        System.out.println("In da else");
        System.out.println(Constants.stateCounter);

        complete = true;
      }
      
    }


}
