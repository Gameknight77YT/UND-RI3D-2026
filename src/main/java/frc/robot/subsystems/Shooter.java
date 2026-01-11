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

  private TalonFX TopShooterMotor = new TalonFX(Constants.TopSchooterMotorID);
  private TalonFX BottomShooterMotor = new TalonFX(Constants.BottomShooterMotorID);

  private TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();
  private TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();

  private InterpolatingDoubleTreeMap topShooterMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap bottomShooterMap = new InterpolatingDoubleTreeMap();

  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  private VelocityVoltage topVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private VelocityVoltage bottomVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  private double targetDistanceMeters = 0.0;
  private Angle targetRelativeAngle = Angle.ofBaseUnits(0, Degree);

  private double topSpeedInterpolatedRPM = 0.0;
  private double bottomSpeedInterpolatedRPM = 0.0;

  /** Creates a new Shooter. */
  public Shooter(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topShooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    topShooterConfig.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    topShooterConfig.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    topShooterConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    topShooterConfig.Slot0.kI = 0; // No output for integrated error
    topShooterConfig.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    topShooterConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    topShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomShooterConfig = topShooterConfig;
    bottomShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TopShooterMotor.getConfigurator().apply(topShooterConfig);
    BottomShooterMotor.getConfigurator().apply(bottomShooterConfig);
    
    for(int i = 0; i < Constants.topSchooterMapPoints.length; i++){
      topShooterMap.put(Constants.topSchooterMapPoints[i][0], Constants.topSchooterMapPoints[i][1]);
    }
    for(int i = 0; i < Constants.bottomSchooterMapPoints.length; i++){
      bottomShooterMap.put(Constants.bottomSchooterMapPoints[i][0], Constants.bottomSchooterMapPoints[i][1]);
    }
    // Populate shooter maps
    //TODO: topShooterMap.put(0,0); done 
    //TODO: bottomShooterMap.put(0,0); done

      
  }

  public void setTargetDistanceMeters(double distanceMeters) {
    targetDistanceMeters = distanceMeters;
  }

  public void updateInterpolatedSpeeds() {
    topSpeedInterpolatedRPM = topShooterMap.get(targetDistanceMeters);
    bottomSpeedInterpolatedRPM = bottomShooterMap.get(targetDistanceMeters);
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
    
    setShooterSpeeds(topSpeedInterpolatedRPM, bottomSpeedInterpolatedRPM);
  }

  public void setShooterSpeeds(double topShooterSpeedRPM, double bottomShooterSpeed) {
    TopShooterMotor.setControl(topVelocityVoltage.withVelocity(topShooterSpeedRPM));
    BottomShooterMotor.setControl(bottomVelocityVoltage.withVelocity(bottomShooterSpeed));
  }

  public void stopShooter() {
    TopShooterMotor.stopMotor();
    BottomShooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetDistanceMeters = getDistToGoal();
    targetRelativeAngle = getAngleToGoal();

    updateInterpolatedSpeeds();

    SmartDashboard.putNumber("Top Shooter Velocity RPM", TopShooterMotor.getVelocity().getValue().in(RevolutionsPerSecond));
    SmartDashboard.putNumber("Bottom Shooter Velocity RPM", BottomShooterMotor.getVelocity().getValue().in(RevolutionsPerSecond));

    SmartDashboard.putNumber("Top Shooter Interpolated RPM", topSpeedInterpolatedRPM);
    SmartDashboard.putNumber("Bottom Shooter Interpolated RPM", bottomSpeedInterpolatedRPM);

  }
}
