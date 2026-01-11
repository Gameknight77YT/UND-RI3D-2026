// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(Constants.IntakeMotorID);
  
  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  /** Creates a new Intake. */
  public Intake() {
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeMotor.getConfigurator().apply(intakeConfig);
    
  }

  public void RunIntake(double intakeSpeed){
      intakeMotor.set(intakeSpeed);
    }
  public void StopMotor(){
      intakeMotor.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
