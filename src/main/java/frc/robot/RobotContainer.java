// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandJoystick manipulatorController = new CommandJoystick(1);
 
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driveController.getRawAxis(translationAxis),
                () -> driveController.getRawAxis(strafeAxis),
                () -> -driveController.getRawAxis(rotationAxis),               
                driveController.x()/*,
                mShooterLimelight,
                mIntakeLimelight*/
            )
        );


    configureBindings();
  }

  private void configureBindings() {
    /* Drive Controller Bindings */
    driveController.y().onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    /* Manipulator Controller Bindings */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
