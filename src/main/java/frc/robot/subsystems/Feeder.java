// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonFX feeder = new TalonFX(Constants.feederMotorID, Constants.CanBus);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();
  /** Creates a new Feeder. */
  public Feeder() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    cfg.CurrentLimits.StatorCurrentLimit = Constants.currentLimit;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.currentLimit;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .2;

    feeder.getConfigurator().apply(cfg);
    
  }

  public void feed(double speed) {
    feeder.set(speed);

  }

  public void stopFeeder() {
    feeder.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
