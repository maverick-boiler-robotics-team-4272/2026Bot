package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;

public class FieldConstants {
    public static final double HUB_HEIGHT_M = 1.83; // There may be a use case, but not at the moment
    public static final double FUEL_MAX_HEIGHT_M = 3; // Ehh, we shall see...
    public static final double ROBOT_MASS_KG = 68.4;
    public static final double FIELD_LENGTH_M = 16.54;
    public static final double FIELD_WIDTH_M = 8.07;

    public static final Pose2d CLIMB_POSE_LEFT = new Pose2d(15.469, 3.297, Rotation2d.fromDegrees(90));
    public static final Pose2d CLIMB_POSE_RIGHT = new Pose2d(15.461, 5.357, Rotation2d.fromDegrees(-90));
    public static ArrayList<Pose2d> CLIMB_POSES = new ArrayList<>(2);
    public static final Pose2d TUNE_POSE = new Pose2d(14.5, 5, Rotation2d.fromDegrees(180));

    public static final Translation2d HUB_LOCATION = new Translation2d(
            !isRedSide() ? 4.75 : FIELD_LENGTH_M - 4.75, 4.035); // yup, I added the extra distance :)

    public static final Pose2d TRENCH_LEFT_OUT_APPROACH = new Pose2d(13.265, 0.697, Rotation2d.kZero);

    public static final Pose2d TRENCH_LEFT_OUT = new Pose2d(12.568, 0.657, Rotation2d.kZero);

    public static final Pose2d TRENCH_LEFT_IN = new Pose2d(11.210, 0.610, Rotation2d.kZero);

    public static final Pose2d TRENCH_LEFT_IN_APPROACH = new Pose2d(10.683, 0.664, Rotation2d.kZero);

    public static final Pose2d TRENCH_RIGHT_OUT_APPROACH = new Pose2d(isRedSide() ? 7.4125 : FIELD_WIDTH_M - 7.4125,
            isRedSide() ? FIELD_LENGTH_M - 3.03 : 3.03, Rotation2d.fromDegrees(180));
    public static final Pose2d TRENCH_RIGHT_OUT = new Pose2d(isRedSide() ? 7.4125 : FIELD_WIDTH_M - 7.4125,
            isRedSide() ? FIELD_LENGTH_M - 4.03 : 4.03, Rotation2d.fromDegrees(180));
    public static final Pose2d TRENCH_RIGHT_IN = new Pose2d(isRedSide() ? 7.4125 : FIELD_WIDTH_M - 7.4125,
            isRedSide() ? FIELD_LENGTH_M - 5.128 : 5.128, Rotation2d.fromDegrees(180));
    public static final Pose2d TRENCH_RIGHT_IN_APPROACH = new Pose2d(isRedSide() ? 7.4125 : FIELD_WIDTH_M - 7.4125,
            isRedSide() ? FIELD_LENGTH_M - 6.128 : 6.128, Rotation2d.fromDegrees(180));

    public static final Pose2d[] LEFT_OUT = { TRENCH_LEFT_OUT_APPROACH, TRENCH_LEFT_OUT, TRENCH_LEFT_IN,
            TRENCH_LEFT_IN_APPROACH };
    public static final Pose2d[] LEFT_IN = { TRENCH_LEFT_IN_APPROACH, TRENCH_LEFT_IN, TRENCH_LEFT_OUT,
            TRENCH_LEFT_OUT_APPROACH };
    public static final Pose2d[] RIGHT_OUT = { TRENCH_RIGHT_OUT_APPROACH, TRENCH_RIGHT_OUT, TRENCH_RIGHT_IN,
            TRENCH_RIGHT_IN_APPROACH };
    public static final Pose2d[] RIGHT_IN = { TRENCH_RIGHT_IN_APPROACH, TRENCH_RIGHT_IN, TRENCH_RIGHT_OUT,
            TRENCH_RIGHT_OUT_APPROACH };

    public static ArrayList<Pose2d> TRENCH_POSES = new ArrayList<>();

    static {
        CLIMB_POSES.add(CLIMB_POSE_RIGHT);
        CLIMB_POSES.add(CLIMB_POSE_LEFT);

        TRENCH_POSES.add(TRENCH_LEFT_OUT_APPROACH);
        TRENCH_POSES.add(TRENCH_LEFT_IN_APPROACH);
        TRENCH_POSES.add(TRENCH_RIGHT_OUT_APPROACH);
        TRENCH_POSES.add(TRENCH_RIGHT_IN_APPROACH);
    }

    public static boolean isRedSide() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return true;
        }
        return false;
    }
}
