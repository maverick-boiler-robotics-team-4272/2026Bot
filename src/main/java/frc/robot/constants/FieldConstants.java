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

  static {
    CLIMB_POSES.add(CLIMB_POSE_RIGHT);
    CLIMB_POSES.add(CLIMB_POSE_LEFT);
  }

  public static final Translation2d HUB_LOCATION = new Translation2d(
      !isRedSide() ? 4.75 : FIELD_LENGTH_M - 4.75, 4.035); // yup, I added the extra distance :)
  public static final Translation2d LEFT_TRENCH_TO_NEUTRAL_ZONE_OUT = new Translation2d(
      isRedSide() ? 6 : FIELD_LENGTH_M - 6,
      isRedSide() ? FIELD_WIDTH_M - 0.5 : 0.5); // TODO: I need actual numbers for this
  public static final Translation2d RIGHT_TRENCH_TO_NEUTRAL_ZONE_OUT = new Translation2d(
      isRedSide() ? 1 : FIELD_LENGTH_M - 1,
      isRedSide() ? 0.5 : FIELD_WIDTH_M - 0.5); // TODO: I need actual numbers for this
  public static final Translation2d LEFT_TRENCH_TO_NEUTRAL_ZONE_IN = new Translation2d(
      isRedSide() ? 6 : FIELD_LENGTH_M - 6,
      isRedSide() ? FIELD_WIDTH_M - 0.5 : 0.5); // TODO: I need actual numbers for this
  public static final Translation2d RIGHT_TRENCH_TO_NEUTRAL_ZONE_IN = new Translation2d(
      isRedSide() ? 6 : FIELD_LENGTH_M - 6,
      isRedSide() ? 0.5 : FIELD_WIDTH_M - 0.5); // TODO: I need actual numbers for this

  public static boolean isRedSide() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return true;
    }
    return false;
  }
}
