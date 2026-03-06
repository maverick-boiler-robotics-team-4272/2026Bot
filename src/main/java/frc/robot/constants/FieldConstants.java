package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

        public static final Pose2d HUB_LOCATION = new Pose2d(
                        isRedSide() ? 11.922 : FIELD_LENGTH_M - 11.922, 4.035, Rotation2d.kZero); // yup, I added the
                                                                                                  // extra distance :)
        public static final Pose2d LEFT_SHUTTLE = new Pose2d(
                        isRedSide() ? 15.821 : FIELD_LENGTH_M - 15.821, isRedSide() ? 1.155 : FIELD_WIDTH_M - 1.155,
                        Rotation2d.kZero);

        public static final Pose2d RIGHT_SHUTTLE = new Pose2d(
                        isRedSide() ? 15.821 : FIELD_LENGTH_M - 15.821, isRedSide() ? FIELD_WIDTH_M - 1.155 : 1.155,
                        Rotation2d.kZero);

        public static ArrayList<Pose2d> SHUTTLE_POSES = new ArrayList<Pose2d>();

        public static final Pose2d TRENCH_LEFT_OUT_APPROACH = new Pose2d(isRedSide() ? 13.265 : FIELD_LENGTH_M - 13.265,
                        isRedSide() ? 0.657 : FIELD_WIDTH_M - 0.657,
                        isRedSide() ? Rotation2d.kZero : Rotation2d.k180deg);

        public static final Pose2d TRENCH_LEFT_OUT = new Pose2d(isRedSide() ? 12.568 : FIELD_LENGTH_M - 12.568,
                        isRedSide() ? 0.657 : FIELD_WIDTH_M - 0.657,
                        isRedSide() ? Rotation2d.kZero : Rotation2d.k180deg);

        public static final Pose2d TRENCH_LEFT_IN = new Pose2d(isRedSide() ? 11.210 : FIELD_LENGTH_M - 11.210,
                        isRedSide() ? 0.657 : FIELD_WIDTH_M - 0.657,
                        isRedSide() ? Rotation2d.kZero : Rotation2d.k180deg);

        public static final Pose2d TRENCH_LEFT_IN_APPROACH = new Pose2d(isRedSide() ? 10.4 : FIELD_LENGTH_M - 10.4,
                        isRedSide() ? 0.657 : FIELD_WIDTH_M - 0.657,
                        isRedSide() ? Rotation2d.kZero : Rotation2d.k180deg);

        public static final Pose2d TRENCH_RIGHT_OUT_APPROACH = new Pose2d(
                        isRedSide() ? 13.265 : FIELD_LENGTH_M - 13.265, isRedSide() ? FIELD_WIDTH_M - 0.657 : 0.657,
                        isRedSide() ? Rotation2d.kZero : Rotation2d.k180deg);
        public static final Pose2d TRENCH_RIGHT_OUT = new Pose2d(isRedSide() ? 12.568 : FIELD_LENGTH_M - 12.568,
                        isRedSide() ? FIELD_WIDTH_M - 0.657 : 0.657,
                        isRedSide() ? Rotation2d.kZero : Rotation2d.k180deg);
        public static final Pose2d TRENCH_RIGHT_IN = new Pose2d(isRedSide() ? 11.210 : FIELD_LENGTH_M - 11.210,
                        isRedSide() ? FIELD_WIDTH_M - 0.657 : 0.657,
                        isRedSide() ? Rotation2d.kZero : Rotation2d.k180deg);
        public static final Pose2d TRENCH_RIGHT_IN_APPROACH = new Pose2d(isRedSide() ? 10.400 : FIELD_LENGTH_M - 10.400,
                        isRedSide() ? FIELD_WIDTH_M - 0.657 : 0.657,
                        isRedSide() ? Rotation2d.kZero : Rotation2d.k180deg);

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

                TRENCH_POSES.add(TRENCH_RIGHT_OUT_APPROACH);
                TRENCH_POSES.add(TRENCH_RIGHT_IN_APPROACH);
                TRENCH_POSES.add(TRENCH_LEFT_OUT_APPROACH);
                TRENCH_POSES.add(TRENCH_LEFT_IN_APPROACH);

                SHUTTLE_POSES.add(LEFT_SHUTTLE);
                SHUTTLE_POSES.add(RIGHT_SHUTTLE);
        }

        public static boolean isRedSide() {
                if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() != DriverStation.Alliance.Red) {
                        return false;
                }
                return true;
        }
}
