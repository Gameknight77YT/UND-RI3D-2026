// package frc.robot.commands;

// import frc.robot.Constants;
// import frc.robot.MathEquations;
// import frc.robot.subsystems.Swerve;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.MiniPID;


// public class TeleopSwerve extends Command {    
//     private Swerve s_Swerve;    
//     private Double translation;
//     private Double strafe;
//     private Double rotation;
//     private DoubleSupplier rotationSup;
//     private DoubleSupplier translationSup;
//     private DoubleSupplier strafeSup;
//     private DoubleSupplier rightYAxis;

//     private BooleanSupplier robotCentricSup;
//     private BooleanSupplier normalTurnSup;
//     double rotationCorrect = 0;
//     double previousYaw;
//     double currentYaw;
//     double gyroChange = 0.0;
//     double goalYaw = 0.0;
//     double previousRotation = 0;
//     double rotationVal = 0;
//     double translationVal;
//     double strafeVal;
//     double stickRotation;
//     private int mHeadingCorrection = 1;

//     private final MathEquations mathEquations = new MathEquations();





//     double kP = 0.01; 
//     double kI = 0;
//     double kD = 0;

//     private final MiniPID mPID = new MiniPID(kP, kI, kD);

//     public TeleopSwerve(Swerve s_Swerve, Double translation, Double strafe, Double rotation, BooleanSupplier normalTurn, BooleanSupplier robotCentricSup, int headingCorrection, DoubleSupplier translationAxis, DoubleSupplier strafeAxis, DoubleSupplier rotationAxis, DoubleSupplier rightyaxis) {
//         this.s_Swerve = s_Swerve;
//         mHeadingCorrection = headingCorrection;
//         addRequirements(s_Swerve);

//         this.translation = translation;
//         this.strafe = strafe;
//         this.rotation = rotation;
//         this.translationSup = translationAxis;
//         this.strafeSup = strafeAxis;
//         this.rotationSup = rotationAxis;
//         this.robotCentricSup = robotCentricSup;
//         this.normalTurnSup = normalTurn;
//         this.rightYAxis = rightyaxis;
//         currentYaw = s_Swerve.getGyroYaw().getDegrees();

//         // SmartDashboard.putNumber("P Gain", kP);
//         // SmartDashboard.putNumber("I Gain", kI);
//         // SmartDashboard.putNumber("D Gain", kD);

//         SmartDashboard.putNumber("P Gain turn", mathEquations.pTurn);
//         SmartDashboard.putNumber("I Gain turn", mathEquations.iTurn);
//         SmartDashboard.putNumber("D Gain turn", mathEquations.dTurn);



        
//     }
                            

//     @Override
//     public void execute() {
//         /* Get Values, Deadband*/
//         //double current_Yaw = Double.parseDouble(s_Swerve.gyro.getYaw().toString());

//         // double p = SmartDashboard.getNumber("P Gain", 0);
//         // double i = SmartDashboard.getNumber("I Gain", 0);
//         // double d = SmartDashboard.getNumber("D Gain", 0);
    
//         // if PID coefficients on SmartDashboard have changed, write new values to controller
//         // if((p != kP)) { mPID.setP(p); kP = p; }
//         // if((i != kI)) { mPID.setI(i); kI = i; }
//         // if((d != kD)) { mPID.setD(d); kD = d; }



//         double p = SmartDashboard.getNumber("P Gain turn", 0);
//         double i = SmartDashboard.getNumber("I Gain turn", 0);
//         double d = SmartDashboard.getNumber("D Gain turn", 0);
      
//         // if PID coefficients on SmartDashboard have changed, write new values to controller
//         if((p != mathEquations.pTurn)) { mathEquations.mTurnPID.setP(p); mathEquations.pTurn = p; }
//         if((i != mathEquations.iTurn)) { mathEquations.mTurnPID.setI(i); mathEquations.iTurn = i; }
//         if((d != mathEquations.dTurn)) { mathEquations.mTurnPID.setD(d); mathEquations.dTurn = d; }

//         previousYaw = currentYaw;
//         currentYaw = s_Swerve.getGyroYaw().getDegrees();
//         while (currentYaw > 360 || currentYaw < -360){
//             currentYaw = currentYaw - 360;
//             if (currentYaw < 0){
//                 currentYaw = 360 - currentYaw;
//             }
//         }

        
//         //currentYaw =
//         //SmartDashboard.putString("yaw acceleration", s_Swerve.gyro.getAngularVelocityYWorld().toString());

//         previousRotation = stickRotation;
//         SmartDashboard.putNumber("headingCorrection", mHeadingCorrection);

//         double stickTranslation = MathUtil.applyDeadband(translationSup.getAsDouble() * mHeadingCorrection, Constants.stickDeadband);
//         double stickStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble() * mHeadingCorrection, Constants.stickDeadband);
//         stickRotation = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
//         double rightStickY = MathUtil.applyDeadband(rightYAxis.getAsDouble(), Constants.stickDeadband);

//         if (normalTurnSup.getAsBoolean()){

//             SmartDashboard.putBoolean("Normal turn?", true);
//             if (stickRotation == 0 && previousRotation != 0){
//                 goalYaw = currentYaw;
//             }

            

//             if (stickRotation == 0 && (strafeVal != 0 || translationVal != 0)){
//                 rotationCorrect = mathEquations.PIDDrivetrainTurn(currentYaw, goalYaw);
//             }

//             rotationVal = 0;

//             if (stickRotation != 0){
//                 rotationVal = stickRotation;
//             }
//             else if (translationVal != 0 || strafeVal != 0){
//                 if (rotationCorrect > 0.1){
//                     rotationCorrect = 0.1;
//                 }
//                 else if (rotationCorrect < -0.1){
//                     rotationCorrect = -0.1; 
//                 }
//                 rotationVal = rotationCorrect;
//             }

//             rotation = rotationVal;
//         }
//         else{
//             SmartDashboard.putBoolean("Normal turn?", false);

//             if (Math.abs(stickRotation) > 0.79 || Math.abs(rightStickY) > 0.79){
//                 double angle = Math.toDegrees(Math.tanh(rightStickY / stickRotation));
//                 if (rightStickY < 0 && stickRotation > 0){
//                     angle = 360 - Math.abs(angle);
//                 }
//                 else if (rightStickY < 0 && stickRotation < 0){
//                     angle = Math.abs(angle);
//                 }
//                 else if (rightStickY > 0 && stickRotation < 0){
//                     angle = 90 + Math.abs(angle);
//                 }
//                 else if (rightStickY > 0 && stickRotation > 0){
//                     angle = angle + 180;
//                 }

//                 if (stickRotation == -1){
//                     angle = 90;
//                 }
//                 if (stickRotation == 1){
//                     angle = 270;
//                 }
//                 if (rightStickY == -1){
//                     angle = 0;
//                 }
//                 if (rightStickY == 1){
//                     angle = 180;
//                 }



//                 goalYaw = angle;

//             }

//             SmartDashboard.putNumber("goal Yaw", goalYaw);
//             SmartDashboard.putNumber("current yaw", currentYaw);
//             SmartDashboard.putNumber("rightStickx", stickRotation);
//             SmartDashboard.putNumber("rightSticky", rightStickY);
//             rotationCorrect = mathEquations.PIDDrivetrainTurn(currentYaw, goalYaw);
//             rotation = rotationCorrect;
//         }


//         // if (commandedRotationVal == 0 && previousRotation != 0){
//         //     goalYaw = currentYaw;
//         // }

//         // SmartDashboard.putNumber("yaw", currentYaw);
        

//         // if (commandedRotationVal == 0 && (strafeVal != 0 || translationVal != 0)){
//         //     rotationCorrect = mPID.getOutput(currentYaw, goalYaw);
//         // }

//         // SmartDashboard.putNumber("rotation goal", goalYaw);
//         // SmartDashboard.putNumber("PID output", rotationCorrect);

//         // rotationVal = 0;

//         // if (commandedRotationVal != 0){
//         //     rotationVal = commandedRotationVal;
//         // }
//         // else if (translationVal != 0 || strafeVal != 0){
//         //     if (rotationCorrect > 0.1){
//         //         rotationCorrect = 0.1;
//         //     }
//         //     else if (rotationCorrect < -0.1){
//         //         rotationCorrect = -0.1; 
//         //     }
//         //     rotationVal = rotationCorrect;
//         // }
//         // SmartDashboard.putNumber("turning", rotationVal);
//         // SmartDashboard.putNumber("goal Yaw", goalYaw);


//         // if (translation != 0){
//         //     translationVal = translation;
//         // }
//         // else{
//         //     translationVal = stickTranslation;
//         // }
//         // if (strafe != 0){
//         //     strafeVal = strafe;
//         // }
//         // else{
//         //     strafeVal = stickStrafe;
//         // }

//         // if (rotation != 0){
//         //     rotationVal = rotation;
//         // }
//         translationVal = stickTranslation;
//         strafeVal = stickStrafe;
//         rotationVal = rotation;

//         /* Drive */
//         s_Swerve.drive(
//             new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
//             rotationVal * Constants.Swerve.maxAngularVelocity, 
//             !robotCentricSup.getAsBoolean(), 
//             true
//         );

//         translation = 0.0;
//         strafe = 0.0;
//         rotation = 0.0;

        
//     }
// }

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.MiniPID;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    double rotationCorrect = 0;
    double previousYaw;
    double currentYaw;
    double gyroChange = 0.0;
    double goalYaw = 0.0;
    double previousRotation = 0;
    double commandedRotationVal = 0;
    double rotationVal = 0;

    //private final ShooterLimelight mShooterLimelight;

    //private final IntakeLimelight mIntakeLimelight;


    private final Joystick mManipulatorJoystick = new Joystick(1);

    private final Joystick driverJoystick = new Joystick(0);

    //private final MathEquations mathEquations = new MathEquations();



    double kP = 0.01; 
    double kI = 0;
    double kD = 0;

    private final MiniPID mPID = new MiniPID(kP, kI, kD);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
        DoubleSupplier rotationSup, BooleanSupplier robotCentricSup/*, 
        ShooterLimelight shooterLimelight, IntakeLimelight intakeLimelight*/) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        currentYaw = s_Swerve.getGyroYaw().getDegrees();
        //mShooterLimelight = shooterLimelight;
        //mIntakeLimelight = intakeLimelight;

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        //mathEquations.mTurnPID.setOutputRampRate(0.1);
        //mathEquations.mTurnPID.setF(0.1);


        
    }
                            

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        //double current_Yaw = Double.parseDouble(s_Swerve.gyro.getYaw().toString());

        double allianceCorrection = 1;
        var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                
                if(alliance.get() == DriverStation.Alliance.Red){
                    SmartDashboard.putBoolean("Alliance", false);
                    allianceCorrection = 1;
                }else{
                    SmartDashboard.putBoolean("Alliance", true);
                    allianceCorrection = -1;
                }
            }
            allianceCorrection = -1;

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        // if((p != kP)) { mPID.setP(p); kP = p; }
        // if((i != kI)) { mPID.setI(i); kI = i; }
        // if((d != kD)) { mPID.setD(d); kD = d; }
        previousYaw = currentYaw;
        currentYaw = s_Swerve.getGyroYaw().getDegrees();
        double counter = 0;
        while (currentYaw > 360 || currentYaw < -360){
            // currentYaw = currentYaw - 360;
            if (currentYaw > 360) {
                currentYaw -= 360;
            } else {
                currentYaw += 360;
            }
            counter += 1;
            if (counter > 5000){
                break;



            }
        }

        
        //currentYaw =
        //SmartDashboard.putString("yaw acceleration", s_Swerve.gyro.getAngularVelocityYWorld().toString());

        previousRotation = commandedRotationVal;

        

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble() * allianceCorrection, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble() * allianceCorrection, Constants.stickDeadband);
        commandedRotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (commandedRotationVal == 0 && previousRotation != 0){
            goalYaw = currentYaw;
        }

        SmartDashboard.putNumber("yaw", currentYaw);
        

        if (commandedRotationVal == 0 && (strafeVal != 0 || translationVal != 0)){
            rotationCorrect = mPID.getOutput(currentYaw, goalYaw);
        }

        SmartDashboard.putNumber("rotation goal", goalYaw);
        SmartDashboard.putNumber("PID output", rotationCorrect);

        rotationVal = 0;

        if (commandedRotationVal != 0){
            rotationVal = commandedRotationVal;
        }
        else if (translationVal != 0 || strafeVal != 0){
            if (rotationCorrect > 0.1){
                rotationCorrect = 0.1;
            }
            else if (rotationCorrect < -0.1){
                rotationCorrect = -0.1; 
            }
            rotationVal = rotationCorrect;
        }

        boolean robotCentric = !robotCentricSup.getAsBoolean();

        /*if (mManipulatorJoystick.getRawButton(10)){
            double ShuttleOffset = s_Swerve.calculateShuttleAngle();
            SmartDashboard.putNumber("Shuttle offset", ShuttleOffset);
            if (Math.abs(ShuttleOffset) > 2){
                double PIDValue = mathEquations.PIDDrivetrainTurn(ShuttleOffset, 0);

                if (Math.abs(PIDValue) > 0.4){
                    PIDValue = 0.4 * (PIDValue/Math.abs(PIDValue));
                }
                SmartDashboard.putNumber("Shuttle PID", PIDValue);
                rotationVal = PIDValue * 1.5;
            }
        }
        
        if (mManipulatorJoystick.getRawButton(1)) {
             // When pressed the intake turns on
             double speakerOffset = mShooterLimelight.getSpeakerCenter();

             if (Math.abs(speakerOffset) > 2){
                double PIDValue = mathEquations.PIDDrivetrainTurn(speakerOffset, 0);

                if (Math.abs(PIDValue) > 0.8){
                    PIDValue = 0.8 * (PIDValue/Math.abs(PIDValue));
                }
                rotationVal = PIDValue;
            }
        }*/

        //double distance = mShooterLimelight.getSpeakerDistance();
    

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            robotCentric, 
            true
        );

        
    }
}