package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;

public class SubsystemConstants {

    public static final String CAN_BUS = "Non-Drive";

    public static class HopperConstants {
        public static final int HOPPER_LOWER_MOTOR_ID = 61;
        public static final int HOPPER_LOWER_MOTOR_2_ID = 62;
        public static final int HOPPER_UPPER_MOTOR_ID = 63;
        public static final int HOPPER_LOWER_SPEED = 60;
        public static final int HOPPER_UPPER_SPEED = 40;

    }

    public static class LoaderConstants {
        public static final int LOADER_MOTOR_1_ID = 31;
        public static final int LOADER_MOTOR_2_ID = 32;
        public static final String LOADER_MOTOR_1_ID_LOG_KEY = "Subsystems/Loader/Loader Motor 1/";
        public static final String LOADER_MOTOR_2_ID_LOG_KEY = "Subsystems/Loader/Loader Motor 2/";
    }

    public static class ShooterConstants {
        public static final int SHOOTER_MOTOR_LEFT_ID = 41;
        public static final int SHOOTER_MOTOR_RIGHT_ID = 42;
        public static final int HOODED_MOTOR_ID = 43;

        public static final String SHOOTER_LOG_KEY = "Subsystems/Shooter/";
        public static final double SHOOTER_WHEEL_RADIUS = 1.5;
        public static final double SHOOTER_WHEEL_CIRCUMFERENCE = Math.PI * SHOOTER_WHEEL_RADIUS;
        public static final double SHOOTER_HEIGHT_M = Meters.convertFrom(16.784, Inches);

        public static final InterpolatingDoubleTreeMap VELOCITY_LOOKUP = InterpolatingDoubleTreeMap
            .ofEntries(Map.entry(0.0, 1.0));
        public static final InterpolatingDoubleTreeMap ANGLE_LOOKUP = InterpolatingDoubleTreeMap
            .ofEntries(Map.entry(0.0, 1.0));
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 51;
        public static final int INTAKE_MOTOR_2_ID = 52;
        public static final double EXTEND_DISTANCE = 40;
        public static final int INTAKE_SPEED = 40;
    }

    public static class DrivetrainConstants {
        public static final double WHEEL_COF = 1.1;//TODO: ask about this
        public static final double MAX_DRIVE_SPEED = 5; //TODO: this needs to be tested
        public static final double MAX_ROTATIONAL_SPEED = 2; //TODO: this needs to be tested
        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(2, MAX_DRIVE_SPEED, WHEEL_COF, DCMotor.getKrakenX60Foc(4), 80, 4);
    }
}
