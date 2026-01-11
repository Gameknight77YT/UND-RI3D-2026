// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class HopperExtender extends SubsystemBase {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private double targetPosition = 0.0;
  /** Creates a new HopperExtender. */
  public HopperExtender() {
    motor = new SparkFlex(Constants.IntakeMotorID, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();
    motorConfig = new SparkFlexConfig();

    motorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(Constants.hopperExtenderPIDP)
      .i(Constants.hopperExtenderPIDI)
      .d(Constants.hopperExtenderPIDD)
      .outputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }

  public double SetPositionTarget(double setTargetPosition){
    //Bounds check
    if(setTargetPosition > Constants.hopperExtenderFullExtendedEncoderPosition){setTargetPosition = Constants.hopperExtenderFullExtendedEncoderPosition;}
    if(setTargetPosition < Constants.hopperExtenderFullExtendedEncoderPosition){setTargetPosition = Constants.hopperExtenderFullRetractedEncoderPosition;}
    targetPosition = setTargetPosition;
    closedLoopController.setReference(setTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    return setTargetPosition;
    }
  public void ResetEncoder(){
      encoder.setPosition(0);
    }

  public double GetTargetPosition(){
    return targetPosition;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
