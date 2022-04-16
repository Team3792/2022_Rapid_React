
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Autonomous.SemiAuto;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
  public boolean finished;
  public int thresholdCounter;
  public PIDController pid = new PIDController(0.0035, .05, 0.002);

  public TurnToAngleCmd(DriveSubsystem drive, Supplier<Double> input) {
    driveTrain = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    //double suppliers init
    this.speedFunction = input;

    finished = false;

    thresholdCounter = 0;

    pid.setIntegratorRange(-0.08, 0.08);
  }

  @Override
  public void initialize() {
   thresholdCounter = 0;
  }
  
  @Override
  public void execute(){

        // Find the heading error; setpoint is given all the time
        double error = SmartDashboard.getNumber("targetAngle", 0);
    
        if (Math.abs(error) > 15) {
          driveTrain.drive(-speedFunction.get(), Math.signum(error) * 0.2, false);
        } else {
          // Turns the robot to face the desired direction
        driveTrain.drive(-speedFunction.get(), -pid.calculate(error, 0), false);
        System.out.println(pid.calculate(error, 0) + "   speed be at");
        System.out.println("error is:" + error);
        }
        
    
        System.out.print("in da execute");
        if (Math.abs(error) < 5) {
          thresholdCounter += 1;
        }

    }

      public void finishCommand(){
        finished = true;
      }

      public void startCommand(){
        finished = false;
      }
      
  @Override
  public boolean isFinished() {
    return thresholdCounter >= 50;
  }
}