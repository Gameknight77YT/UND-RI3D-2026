// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class Intake extends SubsystemBase {
  private SparkFlex motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  /** Creates a new Intake. */
  public Intake() {
    motor = new SparkFlex(Constants.IntakeMotorID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }

  public void RunIntake(double intakeSpeed){
      motor.set(intakeSpeed);
    }
  public void StopMotor(){
      motor.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
