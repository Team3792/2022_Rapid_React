package frc.robot.commands.Autonomous.AutoCommands;

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
  private boolean stageOneComplete = false;

  
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
      if((Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) <  FeetToEncoder(1))){
        //SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
        SmartDashboard.putNumber("Right Drive", driveTrain.rightLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Drive", driveTrain.leftLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Combine", Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2));
        SmartDashboard.putNumber("Correct Val", FeetToEncoder(5));


        // System.out.println("Angle here: " + SmartDashboard.getNumber("targetAngle", 0));
        driveTrain.drive(-1.0, 0, false);


      }


      else if(((Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) >=  FeetToEncoder(1))) && ((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) <  FeetToEncoder(5)){
        //SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
        SmartDashboard.putNumber("Right Drive", driveTrain.rightLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Drive", driveTrain.leftLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Combine", Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2));


        // System.out.println("Angle here: " + SmartDashboard.getNumber("targetAngle", 0));
        // driveTrain.drive(-0.1, SmartDashboard.getNumber("targetAngle", 0), false);
        driveTrain.drive(-0.1, 0, false);


      }
      else
      {
        Constants.stateCounter = 1;
        System.out.println("In da else");
        System.out.println(Constants.stateCounter);
        driveTrain.drive(-0.0, 0.0, false);



        complete = true;
      }
      
    }

    public double FeetToEncoder(double feet) {
      double inches = 12 * feet;
      double wheelRots = inches / 12.566;
      double encoderRots = wheelRots * 7.44;
      double encoderClicks = encoderRots * 2048;
      return encoderClicks;
  
    }

    public void lastAutoDrive()
    {
      while((Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) <  FeetToEncoder(7))){
        //SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
        SmartDashboard.putNumber("Right Drive", driveTrain.rightLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Drive", driveTrain.leftLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Combine", Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2));
        SmartDashboard.putNumber("Correct Val", FeetToEncoder(5));


        // System.out.println("Angle here: " + SmartDashboard.getNumber("targetAngle", 0));
        driveTrain.drive(-0.2, 0, false);


      }
    }

    @Override
    public boolean isFinished() 
    {
      new InstantCommand(AAPowerDistribution::ringLightOff);    
      return complete;
    } 


}
