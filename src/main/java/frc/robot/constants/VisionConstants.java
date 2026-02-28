package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
        public static final String CAMERA_A = "Camera_A";
        public static final String CAMERA_B = "Camera_B";
        public static final String CAMERA_C = "Camera_C";
        public static final String CAMERA_D = "Camera_D";

        public static final Transform3d CAMERA_A_TRANSFORM = new Transform3d(
                        Units.inchesToMeters(12.313), // x
                        Units.inchesToMeters(-12.667), // y
                        Units.inchesToMeters(16.5), // z
                        new Rotation3d(Math.PI /* roll */, 0 /* pitch */, Math.PI / 2 /* yaw */)); // Right side
        public static final Transform3d CAMERA_B_TRANSFORM = new Transform3d(
                        Units.inchesToMeters(12.920),
                        Units.inchesToMeters(12.252),
                        Units.inchesToMeters(15.125),
                        new Rotation3d(Math.PI / 2, -Math.PI / 9, 0)); // Right side shooter // 20 degrees
        public static final Transform3d CAMERA_C_TRANSFORM = new Transform3d(
                        Units.inchesToMeters(12.313),
                        Units.inchesToMeters(12.667),
                        Units.inchesToMeters(16.5),
                        new Rotation3d(Math.PI, 0, -Math.PI / 2)); // left side
        public static final Transform3d CAMERA_D_TRANSFORM = new Transform3d(
                        Units.inchesToMeters(12.920),
                        Units.inchesToMeters(-12.252),
                        Units.inchesToMeters(15.125),
                        new Rotation3d(-Math.PI / 2, -Math.PI / 9, 0)); // left side shooter // 20 degrees

        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVIATIONS = VecBuilder.fill(0.5, 0.5, 3.1415926535897932384);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVIATIONs = VecBuilder.fill(0.2, 0.2, 1.618);
}
