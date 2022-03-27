
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Autonomous.SemiAuto;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** A command that will turn the robot to the specified angle. */
public class TurnToAngleCmd extends CommandBase {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  private final DriveSubsystem driveTrain;
  private final Supplier<Double> speedFunction;
  private double targetHeading;
  public boolean finished;
  public int thresholdCounter;


  public TurnToAngleCmd(DriveSubsystem drive, Supplier<Double> input) {
    driveTrain = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    //double suppliers init
    this.speedFunction = input;

    finished = false;

    thresholdCounter = 0;
  }

  @Override
  public void initialize() {
   thresholdCounter = 0;
  }
  
  @Override
  public void execute(){

       // double kP = 0.0225/(SmartDashboard.getNumber("targetDist", 1.0)*(1/50));
        double kP = 0.00691;
        System.out.println("KP: " + kP);
        // Find the heading error; setpoint is given all the time
        double error = SmartDashboard.getNumber("targetAngle", 0);
    

        // Turns the robot to face the desired direction
        driveTrain.drive(speedFunction.get(), (kP * error), false);
        System.out.println((kP * error) + "   speed be at");
        System.out.println("error is:" + error);
    
        System.out.print("in da execute");

    }

      public void finishCommand(){
        finished = true;
      }

      public void startCommand(){
        finished = false;
      }
      
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return finished;
  }
}