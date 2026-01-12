// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.AllianceUtil;

public class Shooter extends SubsystemBase {
  private final Supplier<Pose2d> poseSupplier;

  private TalonFX shooterMotor = new TalonFX(Constants.shooterMotorID, Constants.CanBus);

  private TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

  private InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  private VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  private double targetDistanceMeters = 0.0;
  private Angle targetRelativeAngle = Angle.ofBaseUnits(0, Degree);

  private double speedInterpolatedRPM = 0.0;

  /** Creates a new Shooter. */
  public Shooter(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    shooterMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.currentLimit;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.currentLimit;
    shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .2;

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    shooterMotorConfig.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    shooterMotorConfig.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    shooterMotorConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    shooterMotorConfig.Slot0.kI = 0; // No output for integrated error
    shooterMotorConfig.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    shooterMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterMotor.getConfigurator().apply(shooterMotorConfig);
    
    for(int i = 0; i < Constants.shooterMapPoints.length; i++){
      shooterMap.put(Constants.shooterMapPoints[i][0], Constants.shooterMapPoints[i][1]);
    }
    // Populate shooter maps
    //TODO: topShooterMap.put(0,0); done 

      
  }

  public void setTargetDistanceMeters(double distanceMeters) {
    targetDistanceMeters = distanceMeters;
  }

  public void updateInterpolatedSpeeds() {
    speedInterpolatedRPM = shooterMap.get(targetDistanceMeters);
  }

  public double getDistToGoal(){
    Pose2d robotPos = poseSupplier.get();
    Pose2d goalPos = AllianceUtil.GetAllianceGoalPos();

    return goalPos
            .relativeTo(robotPos)
            .getTranslation()
            .getNorm();
    }

  public Angle getAngleToGoal(){
    Pose2d robotPos = poseSupplier.get();
    Pose2d goalPos = AllianceUtil.GetAllianceGoalPos();

    return goalPos
        .relativeTo(robotPos)
        .getTranslation()
        .getAngle()
        .getMeasure();
  }

  public void setShooterSpeedsInterpolated() {
    
    setShooterSpeed(speedInterpolatedRPM);
  }

  public void setShooterSpeed(double shooterSpeedRPM) {
    shooterMotor.setControl(shooterVelocityVoltage.withVelocity(shooterSpeedRPM));
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetDistanceMeters = getDistToGoal();
    targetRelativeAngle = getAngleToGoal();

    updateInterpolatedSpeeds();

    SmartDashboard.putNumber("Top Shooter Velocity RPM", shooterMotor.getVelocity().getValue().in(RevolutionsPerSecond));

    SmartDashboard.putNumber("Top Shooter Interpolated RPM", speedInterpolatedRPM);

  }
}
