  package frc.robot.commands.SemiAuto;
  import frc.robot.Constants;
  import frc.robot.subsystems.*;
  import edu.wpi.first.wpilibj2.command.CommandBase;
  import java.util.function.Supplier;
  import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.networktables.NetworkTable;
  import edu.wpi.first.networktables.NetworkTableInstance;
  import edu.wpi.first.wpilibj.Joystick;

public class semiAutoAlignCmd extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


   

  //subsystem declarations
  private final DriveSubsystem driveTrain;
  private boolean complete = false;


  //which camera: 0 is target and 1 is ball
  private final int camera;
  /**
   * Creates a new default drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public semiAutoAlignCmd(DriveSubsystem m_drive, Supplier<Double> input,int camera) {
    driveTrain = m_drive;

    this.camera = camera;

    this.input = input;
     
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);


    
    }

  //joystick input to make sure the driver can still drive forward/back
  private final Supplier<Double> input;

  @Override
  public boolean isFinished() {
    return complete;
  } 


    @Override
    public void execute() {
      if(camera == 0){
        align("targetAngle");
      }
      else{
        align(Constants.ballType + "BallAngle");
        System.out.print(Constants.ballType + "BallAngle");
      }
      
    }

    
    public void align(String camera){
      if(Math.abs(SmartDashboard.getNumber(camera, 0)) > 0.03){
        //SmartDashboard.putNumber("area_gotten", SmartDashboard.getNumber("area", 0));
        driveTrain.drive(input.get(), (SmartDashboard.getNumber(camera, 0))/2);
        System.out.println("Angle here: " + SmartDashboard.getNumber(camera, 0));  
      }
      else
      {
        complete = true;
      }

    }


}

