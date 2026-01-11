package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {
    public static final Pose2d blueGoalLoc = new Pose2d(182.11, 158.84, new Rotation2d()); //Locations based on Welded perimiter found at https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static final Pose2d redGoalLoc = new Pose2d(467.11, 158.84, new Rotation2d()); 
    public static Pose2d GetAllianceGoalPos() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Red) {
            return redGoalLoc;
        }
        return blueGoalLoc;
    }
}
