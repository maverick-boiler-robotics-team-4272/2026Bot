package frc.robot.constants;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;

public class FieldConstants {
        public static final double ROBOT_MASS_KG = 68.4;
        public static final double FIELD_LENGTH_M = 16.54;
        public static final double FIELD_WIDTH_M = 8.07;

        public static final Pose2d RED_HUB_POSE = new Pose2d(FIELD_LENGTH_M - 4.6, 4.035, Rotation2d.kZero);
        public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.6, 4.035, Rotation2d.kZero);

        public static Pose2d getHubLocation() {
                return isRedSide() ? RED_HUB_POSE : BLUE_HUB_POSE;
        }

        public static final Pose2d LEFT_SHUTTLE = new Pose2d(
        isRedSide() ? 15.821 : FIELD_LENGTH_M - 15.821, isRedSide() ? 2.5 :
        FIELD_WIDTH_M - 2.5,
        Rotation2d.kZero);

        public static final Pose2d RIGHT_SHUTTLE = new Pose2d(
        isRedSide() ? 15.821 : FIELD_LENGTH_M - 15.821, isRedSide() ? FIELD_WIDTH_M -
        2.5 : 2.5,
        Rotation2d.kZero);

        public static final Pose2d RED_LEFT_SHUTTLE = new Pose2d(
                15.821, 2.5, Rotation2d.kZero
        );

        public static final Pose2d RED_RIGHT_SHUTTLE = new Pose2d(
                15.821, FIELD_WIDTH_M - 2.5, Rotation2d.kZero
        );

        public static final Pose2d BLUE_LEFT_SHUTTLE = new Pose2d(
                FIELD_LENGTH_M - 15.821, FIELD_WIDTH_M - 2.5, Rotation2d.kZero
        );

        public static final Pose2d BLUE_RIGHT_SHUTTLE = new Pose2d(
                FIELD_LENGTH_M - 15.821, 2.5, Rotation2d.kZero
        );

        public static final ArrayList<Pose2d> RED_SHUTTLE_POSES = new ArrayList<Pose2d>();
        static {
                RED_SHUTTLE_POSES.add(RED_LEFT_SHUTTLE);
                RED_SHUTTLE_POSES.add(RED_RIGHT_SHUTTLE);
        }

        public static final ArrayList<Pose2d> BLUE_SHUTTLE_POSES = new ArrayList<Pose2d>();
        static {
                BLUE_SHUTTLE_POSES.add(BLUE_LEFT_SHUTTLE);
                BLUE_SHUTTLE_POSES.add(BLUE_RIGHT_SHUTTLE);
        }

        public static ArrayList<Pose2d> getShuttlePoses() {
                return isRedSide() ? RED_SHUTTLE_POSES : BLUE_SHUTTLE_POSES;
        }

        public static boolean isRedSide() {
                if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() != DriverStation.Alliance.Red) {
                        return false;
                }
                return true;
        }
}
