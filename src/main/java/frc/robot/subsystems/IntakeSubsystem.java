// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.MotorID.kIntakeMotor);
  private final WPI_TalonSRX intakeToggleMotor = new WPI_TalonSRX(Constants.MotorID.kIntakeToggleMotor);
  // private final DigitalInput topLimitSwitch = new DigitalInput(0);
  // private final DigitalInput bottomLimitSwitch = new DigitalInput(1);
  
  public IntakeSubsystem() {}

  public void setValue(double v)
  {
    intakeMotor.set(v);
  }



  public void stopSubsystemIntake()
  {
    intakeMotor.set(0);
  }

  public void dropIntake()
  {
    intakeToggleMotor.set(0.40);
  }

  public void raiseIntake()
  {
    intakeToggleMotor.set(-0.40);
  }

  public void toggleIntake()
  {
    if (intakeToggleMotor.getSensorCollection().isFwdLimitSwitchClosed())
    {
      
    }

  }

  public void stopTheMadness()
  {
    intakeToggleMotor.set(0);

  }

  // public void dropIntake()
  // {
  //   if(topLimitSwitch.get())
  //   {
  //     intakeToggleMotor.set(0.3);
  //   }
  //   else
  //   {
  //     intakeToggleMotor.set(0);
  //   }
  // }

  

}