package frc.robot.commands.Autonomous.AutoCommands;

import frc.robot.AAPowerDistribution;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.LightsCmd;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;



public class PathToTerminal extends CommandBase{
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
  public PathToTerminal(DriveSubsystem driveTrain) {
    this.driveTrain = driveTrain;

     
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);


    
    }
    @Override
    public void initialize() 
    {
    }


    @Override
    public void execute() {
      
      
      
      
      
      
      
      
      complete = true;
      
      
    }

    public double FeetToEncoder(double feet) {
      double inches = 12 * feet;
      double wheelRots = inches / 12.566;
      double encoderRots = wheelRots * 7.44;
      double encoderClicks = encoderRots * 2048;
      return encoderClicks;
  
    }

    
    @Override
    public boolean isFinished() 
    {
      new InstantCommand(AAPowerDistribution::ringLightOff);    
      return complete;
    } 


}
