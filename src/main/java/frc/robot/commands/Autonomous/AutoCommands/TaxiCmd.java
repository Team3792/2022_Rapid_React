package frc.robot.commands.Autonomous.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AAPowerDistribution;
import frc.robot.subsystems.DriveSubsystem;


public class TaxiCmd extends CommandBase{
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
  public TaxiCmd(DriveSubsystem driveTrain) {
    this.driveTrain = driveTrain;

     
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);


    
    }
    @Override
    public void initialize() 
    {
      // new InstantCommand(AAPowerDistribution::ringLightOn);  
      driveTrain.zeroSensors();
      complete = false;
      System.out.println("We are here at the fucking taxi command");
    }


    @Override
    public void execute() {
      if((Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) <  FeetToEncoder(3))){
        //SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
        SmartDashboard.putNumber("Right Drive", driveTrain.rightLead.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Drive", driveTrain.leftLead.getSelectedSensorPosition());
        
        SmartDashboard.putNumber("Combine", Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2));
        SmartDashboard.putNumber("Correct Val", FeetToEncoder(3));


        // System.out.println("Angle here: " + SmartDashboard.getNumber("targetAngle", 0));
        driveTrain.drive(1.0, 0, false);
        System.out.println("Reached Taxi Speed");


      }
      else{
        System.out.println("In da else");
        driveTrain.drive(0.0, 0.0, false);



        complete = true;
      }

      // else if(((Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) >=  FeetToEncoder(1))) 
      //       && ((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) <  FeetToEncoder(3)){
      //   //SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
      //   SmartDashboard.putNumber("Right Drive", driveTrain.rightLead.getSelectedSensorPosition());
      //   SmartDashboard.putNumber("Left Drive", driveTrain.leftLead.getSelectedSensorPosition());
      //   SmartDashboard.putNumber("Combine", Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2));


      //   // System.out.println("Angle here: " + SmartDashboard.getNumber("targetAngle", 0));
      //   // driveTrain.drive(-0.1, SmartDashboard.getNumber("targetAngle", 0), false);
      //   driveTrain.drive(0.1, 0, false);
      //   System.out.println("Reached Taxi Slow");


      // }

      // else if(((Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) >=  FeetToEncoder(3))) 
      //         && ((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2) <  FeetToEncoder(5)){
      //   //SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
      //   SmartDashboard.putNumber("Right Drive", driveTrain.rightLead.getSelectedSensorPosition());
      //   SmartDashboard.putNumber("Left Drive", driveTrain.leftLead.getSelectedSensorPosition());
      //   SmartDashboard.putNumber("Combine", Math.abs((Math.abs(driveTrain.leftLead.getSelectedSensorPosition()) + Math.abs(driveTrain.rightLead.getSelectedSensorPosition())) / 2));


      //   // System.out.println("Angle here: " + SmartDashboard.getNumber("targetAngle", 0));
      //   // driveTrain.drive(-0.1, SmartDashboard.getNumber("targetAngle", 0), false);
      //   driveTrain.drive(1.0, 0, false);


      // }
      
      
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
