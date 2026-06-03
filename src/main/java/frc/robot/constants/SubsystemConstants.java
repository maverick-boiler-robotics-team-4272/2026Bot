package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Map;

public class SubsystemConstants {

  public static final String CAN_BUS = "CAN'Train";

  public static class HopperConstants {
    public static final int HOPPER_LOWER_MOTOR_ID = 61;
    public static final int HOPPER_LOWER_MOTOR_2_ID = 62;
    public static final int HOPPER_UPPER_MOTOR_ID = 60;

    public static final String HOPPER_KEY = "Subsystems/Hopper/";

    public static final double HOPPER_LOWER_SPEED = 25;
    public static final double HOPPER_LOWER_MAX_SPEED = 50; //Yeah Yeah... I know it is like 46, but this is a nice number to work with
    public static final double HOPPER_UPPER_SPEED = -50;
  }

  public static class LoaderConstants {
    public static final int LOADER_MOTOR_1_ID = 31;
    public static final int LOADER_MOTOR_2_ID = 32;

    public static final double LOADER_SPEED = 50;
    public static final String LOADER_KEY = "Subsystems/Loader/";
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_ID = 41;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 42;
    public static final int HOODED_MOTOR_ID = 43;

    public static final String SHOOTER_LOG_KEY = "Subsystems/Shooter/";

    public static final double AUTO_SHOOTER_HOOD = 0.02;
    public static final double AUTO_SHOOTER_VELOCITY = 55;

    public static final InterpolatingDoubleTreeMap SHOOTER_VELOCITY_LOOKUP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.0, 36.686171), Map.entry(3.07, 51.0), Map.entry(4.95, 64.0));
      
    public static final InterpolatingDoubleTreeMap SCORE_ANGLE_LOOKUP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.942, 0.0), Map.entry(3.07, 0.015), Map.entry(4.95, 0.04));

    public static final InterpolatingDoubleTreeMap TufF_TABLE = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.0, 0.94)/*FAKE*/, Map.entry(2.0, 1.1), Map.entry(3.87, 1.2), Map.entry(4.98, 1.5), Map.entry(7.0, 1.7)/*FAKE*/);
   
    public static final InterpolatingDoubleTreeMap TufF_TABLE_SHUTTLE = InterpolatingDoubleTreeMap.ofEntries(
    Map.entry(1.0, 0.94)/*FAKE*/, Map.entry(2.0, 1.1), Map.entry(3.87, 1.2), Map.entry(4.98, 1.5), Map.entry(7.0, 1.7)/*FAKE*/);

    public static final InterpolatingDoubleTreeMap SHUTTLE_SPEED_TABLE = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(6.0, 40.0), Map.entry(8.9, 60.0), Map.entry(10.0, 80.0));
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 51;
    public static final int EXTENSION_MOTOR_ID = 52;
    public static final int EXTENSION_MOTOR_I2_D = 53;

    public static final double EXTEND_DISTANCE = 10.56;
    public static final int INTAKE_SPEED = 50;
    public static final String INTAKE_KEY = "Subsystems/Intake/";
  }

  public static class DrivetrainConstants {
    public static final double WHEEL_COF = 1.1;
    public static final double MAX_DRIVE_SPEED = 5; 
    public static final double MAX_ROTATIONAL_SPEED = 15;
    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(2, MAX_DRIVE_SPEED, WHEEL_COF,
        DCMotor.getKrakenX60Foc(4), 50, 4);

    public static final double DRIVE_P = 4.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.01;

    public static final double ROTATION_P = 6.25;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
  }
}
