package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    public static final double HUB_HEIGHT_M = 1.83;
    public static final double FUEL_MAX_HEIGHT_M = 3;
    public static final double ROBOT_MASS_KG = 68.4;

    public static final Translation2d HUB_LOCATION = new Translation2d(isRedSide() ? 12.51 : 4.03, 4.035);
    public static final Pose2d CLIMB_POSE = new Pose2d(0,0,Rotation2d.fromDegrees(0));//TODO: I need actual numbers for this
    
    public static boolean isRedSide() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            return true;
        }
        return false;
    }
}
