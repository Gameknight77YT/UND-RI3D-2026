package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.FollowPathHolonomic;
//import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.LimelightHelpers;


public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private final Field2d mField = new Field2d();

    private final Field2d mField2 = new Field2d();

    //private final AutoBuilder teleopAutoBuilder = new AutoBuilder();

    public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(0.4064, 0.4064), 
        new Translation2d(0.381, -0.381), 
        new Translation2d(-0.381, 0.381), 
        new Translation2d(-0.381, -0.381));

    public final SwerveDrivePoseEstimator m_poseEstimator;

    
   


    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "CTREDevicesCanivore");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        SmartDashboard.putData("Field", mField);

        SmartDashboard.putData("Field2", mField2);

        

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };


        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        //this.configurePathPlanner();

        m_poseEstimator = new SwerveDrivePoseEstimator(
          m_kinematics,
          new Rotation2d(0),
          getModulePositions(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public Pose2d getEstimatedPosition(){
        return m_poseEstimator.getEstimatedPosition();
    }

    /*public Command PathFindThenFollowPathTeleop(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                5.76, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // See the "Follow a single path" example for more info on what gets passed here
        return new PathfindThenFollowPathHolonomic(
                path,
                constraints,
                this::getEstimatedPosition,
                this::getCurrentRobotChassisSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(new PIDConstants(15, 0, 0),
                                            new PIDConstants(2.7, 0, 0),
                                            5.76072,
                                            0.4064,
                                            new ReplanningConfig()), // HolonomicPathFollwerConfig, see the API or "Follow a single path" example for more info
                0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to drive subsystem to set requirements
        );
    }

    public Command FollowPathTeleop(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    //PathPlannerPath.fromPathFile("")

    //new FollowPathHolonomic(path, null, null, null, null, null, null)

    return new FollowPathHolonomic(path, 
    this::getEstimatedPosition,
    this::getCurrentRobotChassisSpeeds, 
    this::driveRobotRelative,
    new HolonomicPathFollowerConfig(new PIDConstants(15, 0, 0),
                                            new PIDConstants(2.7, 0, 0),
                                            5.76072,
                                            0.4064,
                                            new ReplanningConfig()),
    ()->{

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
    this);


    }

    private void configurePathPlanner() {
        // double driveBaseRadius = 0;
        // for (var moduleLocation : m_moduleLocations) {
        //     driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        // }


        AutoBuilder.configureHolonomic(
            ()->this.getPose(), // Supplier of current robot pose
            this::setPose,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.driveRobotRelative(speeds), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(15, 0, 0),
                                            new PIDConstants(2.7, 0, 0),
                                            5.76072,
                                            0.4064,
                                            new ReplanningConfig()),
            ()->{

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }*/

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }
  

    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds));
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(m_poseEstimator.getEstimatedPosition().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue().in(Degrees));
        //Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public double getPoseEstimateHeading(){
        return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    /*public double calculateShuttleAngle(){
        double xPos = m_poseEstimator.getEstimatedPosition().getX();
        double yPos = m_poseEstimator.getEstimatedPosition().getY();

        if (DriverStation.getAlliance().isPresent()){

            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                double xOffset = 14.5 - xPos;
                double yOffset = 6.5 - yPos;
                SmartDashboard.putNumber("heading", getPoseEstimateHeading());
                SmartDashboard.putNumber("calculated angle", (180 - Math.toDegrees(Math.atan(yOffset/xOffset))));
                if (xOffset != 0){
                    double offset = getPoseEstimateHeading() + (180 - Math.toDegrees(Math.atan(yOffset/xOffset)));
                    if (Math.abs(offset) > 180){
                        offset = (Math.abs(offset) - 360) * (offset/Math.abs(offset));
                        SmartDashboard.putNumber("Corrected final offset", offset);
                    }


                    return offset;
                }
                else{
                    return 0;
                }
            }

            else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
                double xOffset = xPos - 2.67;
                //double yOffset = yPos - 3.45;
                double yOffset = 6.5 - yPos;
                if (xOffset != 0){
                    double offset = getPoseEstimateHeading() + (Math.toDegrees(Math.atan(yOffset/xOffset)));
                    if (Math.abs(offset) > 180){
                        offset = (Math.abs(offset) - 360) * (offset/Math.abs(offset));
                    }
                    return offset;
                }
                else{
                    return 0;
                }
            }
        }
        return 0;

    }

    public double calculateShuttleDistance(){
        double xPos = m_poseEstimator.getEstimatedPosition().getX();
        double yPos = m_poseEstimator.getEstimatedPosition().getY();
        if (DriverStation.getAlliance().isPresent()){
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                double xOffset = 14.5 - xPos;
                double yOffset = 6.5 - yPos;

                return (Math.sqrt(Math.pow(xOffset, 2) + Math.pow(yOffset, 2))) + 1.5;
            }

            else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
                double xOffset = xPos - 2.67;
                double yOffset = yPos - 3.45;
                return (Math.sqrt(Math.pow(xOffset, 2) + Math.pow(yOffset, 2))) + 1.5;
            }
        }

        return 0;
    }*/


    public void updateOdometry() {
    m_poseEstimator.update(
        gyro.getRotation2d(),
        getModulePositions());


    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("getpipe").getInteger(0) != 0){
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight-shooter", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
      if(Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()/*gyro.getRate() */) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("getpipe").getInteger(0) != 0){
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }


    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        mField.setRobotPose(swerveOdometry.getPoseMeters());

        updateOdometry();

        mField2.setRobotPose(m_poseEstimator.getEstimatedPosition());

        SmartDashboard.putNumber("Drivetrain Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        SmartDashboard.putNumber("Drivetrain x position", m_poseEstimator.getEstimatedPosition().getX());

        SmartDashboard.putNumber("Drivetrain y position", m_poseEstimator.getEstimatedPosition().getY());

        //SmartDashboard.putNumber("Shuttle distance", calculateShuttleDistance());

        //SmartDashboard.putNumber("Shuttle angle", calculateShuttleAngle());


        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}