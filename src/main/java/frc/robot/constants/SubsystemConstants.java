package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Map;

public class SubsystemConstants {

  public static final String CAN_BUS = "Drivetrain";

  public static class HopperConstants {
    public static final int HOPPER_LOWER_MOTOR_ID = 61;
    public static final int HOPPER_LOWER_MOTOR_2_ID = 62;
    public static final int HOPPER_UPPER_MOTOR_ID = 60;

    public static final String HOPPER_KEY = "Subsystems/Hopper/";

    public static final int HOPPER_LOWER_SPEED = 50;
    public static final int HOPPER_UPPER_SPEED = -20;
  }

  public static class LoaderConstants {
    public static final int LOADER_MOTOR_1_ID = 31;
    public static final int LOADER_MOTOR_2_ID = 32;

    public static final String LOADER_KEY = "Subsystems/Loader/";
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_ID = 41;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 42;
    public static final int HOODED_MOTOR_ID = 43;

    public static final String SHOOTER_LOG_KEY = "Subsystems/Shooter/";
    public static final double SHOOTER_WHEEL_RADIUS = 1.5;
    public static final double SHOOTER_WHEEL_CIRCUMFERENCE = Math.PI * SHOOTER_WHEEL_RADIUS;
    public static final double SHOOTER_HEIGHT_M = Meters.convertFrom(16.784, Inches);
    public static final double IDLE_SPEED = 0;

    public static final double AUTO_SHOOTER_HOOD = 0.02;
    public static final double AUTO_SHOOTER_VELOCITY = 55;

    public static final double SHOOTER_SHUTTLE_VELOCITY = 55; // TODO: tune

    public static final InterpolatingDoubleTreeMap SHOOTER_VELOCITY_LOOKUP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(2.26, 38.0), Map.entry(5.03, 58.5));// Map.entry(3.19, 45.0), Map.entry(5.03, 57.0), Map.entry(3.35,
                                                      // 47.0));
    public static final InterpolatingDoubleTreeMap SCORE_ANGLE_LOOKUP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(2.26, 0.0084), Map.entry(5.03, 0.032));// Map.entry(3.19, 0.016), Map.entry(5.03, 0.032),
                                                         // Map.entry(3.35, 0.018));
    // public static final InterpolatingDoubleTreeMap SHUTTLE_ANGLE_LOOKUP =
    // InterpolatingDoubleTreeMap
    // .ofEntries(Map.entry(1.0, 1.0), Map.entry(2.0, 2.0), Map.entry(3.0, 3.0),
    // Map.entry(4.0, 4.0));
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 51;
    public static final int INTAKE_MOTOR_2_ID = 52;

    public static final double EXTEND_DISTANCE = 16.4;
    public static final int INTAKE_SPEED = 50;
    public static final String INTAKE_KEY = "Subsystems/Intake/";
  }

  public static class ClimberConstants {
    // public static final int CLIMBER_MOTOR_ID = 71;
    // public static final String CLIMBER_KEY = "Subsystems/Climber/";
    // public static final int UNLATCH_ROTATIONS = 10; // : tune
    // public static final int CLIMB_ROTATIONS = 77; // : Tune
  }

  public static class DrivetrainConstants {
    public static final double WHEEL_COF = 1.1;
    public static final double MAX_DRIVE_SPEED = 10; // TODO: this needs to be tested
    public static final double MAX_ROTATIONAL_SPEED = 7.5; // TODO: this needs to be tested
    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(2, MAX_DRIVE_SPEED, WHEEL_COF,
        DCMotor.getKrakenX60Foc(4), 80, 4);

    public static final double DRIVE_P = 4.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.01;

    public static final double ROTATION_P = 10.0;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
  }
}
