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
        Units.inchesToMeters(12.318),//x
        Units.inchesToMeters(12.627),//y
        Units.inchesToMeters(16.553),//z
        new Rotation3d(0/*roll*/, 0/*pitch*/, 90/*yaw*/)
    ); //Right side
    public static final Transform3d CAMERA_B_TRANSFORM = new Transform3d(
        Units.inchesToMeters(-12.318),
        Units.inchesToMeters(12.627),
        Units.inchesToMeters(16.553),
        new Rotation3d(0,0,270)
    ); //Left side
    public static final Transform3d CAMERA_C_TRANSFORM = new Transform3d(); //right shooter
    public static final Transform3d CAMERA_D_TRANSFORM = new Transform3d(); //left shooter

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVIATIONS = VecBuilder.fill(0.5, 0.5, 3.14159265358979323846264338327950288419716939937901);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVIATIONs = VecBuilder.fill(0.2, 0.2, 2.71828);
}
