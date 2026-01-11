package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {
    public static final Pose2d blueGoalLoc = new Pose2d(0.0, 5.55, new Rotation2d());
    public static final Pose2d redGoalLoc = new Pose2d(0.0, 5.55, new Rotation2d());
    public static Pose2d GetAllianceGoalPos() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Red) {
            return redGoalLoc;
        }
        return blueGoalLoc;
    }
}
