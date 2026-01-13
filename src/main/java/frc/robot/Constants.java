package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class Constants {
    public static final double stickDeadband = 0.1;
    public static final String CanBus = "CTREDevicesCanivore";

    // Motor IDs
    public static final int shooterMotorID = 14;
    public static final double shooterMotorPIDP = 0.1;
    public static final double shooterMotorPIDI = 0.0;
    public static final double shooterMotorPIDD = 0.0;

    public static final double shooterAutoAimP = 0.02;
    public static final double shooterAutoAimI = 0.0;
    public static final double shooterAutoAimD = 0.0;

    public static final double shooterMapPoints[][] = {
        {1.703, 2.01, 2.29, 2.61, 2.95, 3.45, 4.05, 4.5, 5.08, 5.6, 6.31},
        {41.0, 42.0, 43.0, 48.0, 50.0, 51.0, 55.5, 57.0, 61.0, 64.0, 70.0}}; 
        //First array is distance values in meters to the goal, second array is target values for the shooter

    public static final double shooterBotOffset = 0.3; // dist in meters between center of the bot and the center of the shooter. 

    public static final double defaultDistToGoal = 2;
    public static final double minimumDistToGoal = 0.2; //if for some reason dist to goal is calculated as being less or greater than these distances, it will default to defaultDistToGoal
    public static final double maxDistToGoal = 10;

    public static final int IntakeMotorID = 15;
    public static final double intakeMotorPercentPower = 0.95;

    public static final int hopperMotorID = 16;
    public static final double hopperExtenderPIDP = 0.1;
    public static final double hopperExtenderPIDI = 0.0;
    public static final double hopperExtenderPIDD = 0.0;
    public static final double hopperExtenderFullExtendedEncoderPosition = -20.0;
    public static final double hopperExtenderFullRetractedEncoderPosition = 0.0;
    public static final double hopperExtenderEntensionPosIncrement = -0.2;
    public static final double hopperExtenderRetractionPosIncrement = 0.2;

    public static final int feederMotorID = 17;
    public static final double feederSpeed = .95;
    
    public static final double currentLimit = 35;
    public static final double shooterMaxRPM = 75;

    
public static final class Swerve {
        public static final int pigeonID = 13;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(5.116); // 5.3571

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

            
        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.76; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double angularVelocityMultiplier = 5.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-102.3+180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-29.4+180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-71.98);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(67.5+180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

}
