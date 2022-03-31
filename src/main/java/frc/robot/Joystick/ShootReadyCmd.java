// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Joystick;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import org.opencv.core.Mat;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// // import frc.robot.subsystems.ExampleSubsystem;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;


// /** An example command that uses an example subsystem. */
// public class ShootReadyCmd extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


//   private final WPI_TalonFX shooter = new WPI_TalonFX(Constants.MotorID.kShootMotor);
//   private final WPI_TalonSRX roller = new WPI_TalonSRX(Constants.MotorID.kRollerMotor);

//   private final PS5Mapping controller = new PS5Mapping();

//   private double shootSetpoint;
//   private double rollerSetpoint;

//   private double shooterRPM;
//   private double rollerRPM;

//   private boolean visionShoot;
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public ShootReadyCmd(double shootSetpoint, double rollerSetpoint, boolean visionShoot) 
//   {

//     this.shootSetpoint = shootSetpoint;
//     this.rollerSetpoint = rollerSetpoint;
//     this.visionShoot = visionShoot;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() 
//   {
    
//   }

 

//   public double toRollerRPM(double rawV) 
//   {
//       return (rawV * 600 / 4096);
//   }

//   private double toShooterRPM(double rawV) 
//   {
//     return (rawV * 600 * 2 / 2048);
//   }

  

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() 
//   {

//     rollerRPM = toRollerRPM(roller.getSelectedSensorVelocity());
//     shooterRPM = toShooterRPM(shooter.getSelectedSensorVelocity());

//     if (visionShoot)
//     {
//       if ((Math.abs(rollerRPM - getVisionRollerRPM())) < 25 && (Math.abs(shooterRPM-getVisionShooterRPM())) < 25)
//       {
//         controller.startIntenseShake();
//         System.out.println("SHAKE");
//       }
//       else
//       {
//         controller.stopShake();
//       }
//     }
//     else if (!visionShoot)
//     {
//       if ((Math.abs(rollerRPM - rollerSetpoint)) < 25 && (Math.abs(shooterRPM-shootSetpoint)) < 25)
//       {
//         controller.startIntenseShake();
//       }
//       else
//       {
//         controller.stopShake();
//       }
//     }
//   }


//   public double getVisionShooterRPM() {
//     double xVal = SmartDashboard.getNumber("targetDist", 50);
//     System.out.println("XVAL " + xVal);
//     if (xVal <= 50)
//     {
//         return -0.0069879149* Math.pow(xVal, 3) 
//                 + 0.9253397123 * Math.pow(xVal, 2) 
//                 - 36.6293462365 * xVal 
//                 + 5393.3848674349;
//     }
//     else if (xVal > 50 && xVal <= 93) {
//         return 0.0000414414259424234 * Math.pow(xVal, 5)
//                 - 0.0149920706919886  * Math.pow(xVal, 4)
//                 + 2.13797087428725  * Math.pow(xVal, 3)
//                 - 150.151797625819  * Math.pow(xVal, 2)
//                 + 5202.12230744896  * Math.pow(xVal, 1)
//                 - 68255.4031903005;
//     } else {
//         return 0.00105705403208844 * Math.pow(xVal, 4)
//                 - 0.473451255106056 * Math.pow(xVal, 3)
//                 + 78.9367234383612  * Math.pow(xVal, 2)
//                 - 5795.36686845961  * Math.pow(xVal, 1)
//                 + 160476.696335883;
//     }
// }

//   public double getVisionRollerRPM() {
//     double xVal = SmartDashboard.getNumber("targetDist", 50);
//     double kShootVal;
//     if (xVal <= 93) {
//         kShootVal = 0.0000414414259424234 * Math.pow(xVal, 5) 
//                 - 0.0149920706919886  * Math.pow(xVal, 4)  
//                 + 2.13797087428725  * Math.pow(xVal, 3)  
//                 - 150.151797625819  * Math.pow(xVal, 2)  
//                 + 5202.12230744896  * Math.pow(xVal, 1)  
//                 - 68255.4031903005;
//     } else {
//         kShootVal = 0.00105705403208844 * Math.pow(xVal, 4) 
//                 - 0.473451255106056 * Math.pow(xVal, 3) 
//                 + 78.9367234383612  * Math.pow(xVal, 2) 
//                 - 5795.36686845961  * Math.pow(xVal, 1) 
//                 + 160476.696335883;
//     }
//     double friction = 0.45;
//     double OmegaT;
//     double vVal;
//     double OmegaB;
//     double RPM; 
//     xVal += 47;

//     OmegaB = (kShootVal*(2*Math.PI)/60); 
//     vVal = (xVal)/(0.3907311285*Math.sqrt((-81.5+2.36*xVal)/192)); 
//     System.out.println("vVal: " + vVal); 
//     OmegaT = -(vVal-2*OmegaB*2*Math.PI)/(friction*2*Math.PI); 
//     RPM = (OmegaT*60)/(2*Math.PI); 
//     if (xVal < 97){
//       return -0.1*RPM;
//     }
//     else if (xVal<140) { 
//       return 0.7*RPM; 
//     } else{ 
//       return 1.2*RPM; 
//     }
//   }


//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }