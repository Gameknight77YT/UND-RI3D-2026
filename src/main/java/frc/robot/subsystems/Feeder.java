// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonFX feeder = new TalonFX(Constants.feederMotorID, Constants.CanBus);

  /* Start at velocity 0, use slot 0 */
  private VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();
  /** Creates a new Feeder. */
  public Feeder() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    cfg.CurrentLimits.StatorCurrentLimit = Constants.currentLimit;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.currentLimit;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .2;

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    cfg.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    cfg.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    cfg.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    cfg.Slot0.kI = 0; // No output for integrated error
    cfg.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    feeder.getConfigurator().apply(cfg);
    
  }

  public void feed(double speed) {
    feeder.setControl(feederVelocityVoltage.withVelocity(speed*70));

  }

  public void stopFeeder() {
    feeder.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Feeder speed", feeder.getVelocity().getValue().in(RevolutionsPerSecond));
  }
}
