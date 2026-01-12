// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.HopperExtender;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController manipulatorController = new CommandXboxController(1);
 
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;


  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Shooter shooter = new Shooter(swerve::getEstimatedPosition);
  private final Intake intake = new Intake();
  private final HopperExtender hopperExtender = new HopperExtender();
  
  public RobotContainer() {
    swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> driverController.getRawAxis(translationAxis),
                () -> driverController.getRawAxis(strafeAxis),
                () -> -driverController.getRawAxis(rotationAxis),               
                driverController.x(),
                driverController.leftTrigger(.1)
                /*,
                mShooterLimelight,
                mIntakeLimelight*/
            )
        );


    configureBindings();
  }

  private void configureBindings() {
    /* Drive Controller Bindings */
    driverController.y().onTrue(Commands.runOnce(() -> swerve.zeroHeading()));

    driverController.rightTrigger(.1).whileTrue(//Run intake forward
      intake.runEnd(
        () -> intake.RunIntake(Constants.intakeMotorPercentPower),
        () -> intake.StopMotor()
      ));

    driverController.rightBumper().whileTrue( //Run intake in reverse
      intake.runEnd(
        () -> intake.RunIntake(-Constants.intakeMotorPercentPower),
        () -> intake.StopMotor()
      ));
    /* Manipulator Controller Bindings */
    manipulatorController.a().whileTrue(
      shooter.runEnd(
        () -> shooter.setShooterSpeedsInterpolated(), 
        () -> shooter.stopShooter()
    ));

    manipulatorController.rightBumper().whileTrue(
      hopperExtender.run(
        () -> hopperExtender.IncrementPositionTarget(Constants.hopperExtenderEntensionPosIncrement)
    ));

    manipulatorController.leftBumper().whileTrue(
      hopperExtender.run(
        () -> hopperExtender.IncrementPositionTarget(Constants.hopperExtenderRetractionPosIncrement)
    ));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
