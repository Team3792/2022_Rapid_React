package frc.robot.commands.AutoRoutines;

import frc.robot.AAPowerDistribution;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.LightsCmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
  // private PDHSubsystem m_PDH = new PDHSubsystem();

  private boolean complete = false;

  
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlignCmd(DriveSubsystem driveTrain) {
    this.driveTrain = driveTrain;

     
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);


    
    }
    @Override
    public void initialize() 
    {
      new InstantCommand(AAPowerDistribution::ringLightOn);    
    }


    @Override
    public void execute() {

      if(Math.abs(SmartDashboard.getNumber("targetAngle", 0)) > 0.07){
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
    @Override
    public boolean isFinished() 
    {
      new InstantCommand(AAPowerDistribution::ringLightOff);    
      return complete;
    } 


}
