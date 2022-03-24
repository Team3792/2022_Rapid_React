package frc.robot.commands.Autonomous.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.IntakeCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class AutoDriveCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

   //subsystem declaration
   private final DriveSubsystem driveTrain;
   private final IntakeSubsystem intake;
 
   //intake command declaration
   private final IntakeCmd intakeCmd;

   private boolean complete;

 
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveCmd(DriveSubsystem subsystem, IntakeSubsystem intake) {
    driveTrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.intake = intake;
    intakeCmd = new IntakeCmd(intake);
    intakeCmd.runIntakeForward();
    complete = false;
    
    }
    @Override
    public void execute() {

      // SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
      if(SmartDashboard.getNumber(Constants.ballType + "BallArea",0)<4500 && SmartDashboard.getNumber(Constants.ballType + "BallArea",0)!=0)
      {
        driveTrain.drive(-0.3, (SmartDashboard.getNumber(Constants.ballType + "BallAngle", 0))/3, false);
        System.out.println("Angle here: " + SmartDashboard.getNumber(Constants.ballType + "BallAngle", 0));   
      }
      else{
        //intakeCmd.stopIntake();
        complete = true; 

      }

    }
    @Override
    public boolean isFinished() {
      return complete;
  } 
}
