package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.system.plant.DCMotor;

public class SubsystemConstants {

    public static final String CAN_BUS = "Drivetrain";

    public static class HopperConstants {
        public static final int HOPPER_LOWER_MOTOR_ID = 21;
        public static final int HOPPER_LOWER_MOTOR_2_ID = 23;
        public static final int HOPPER_UPPER_MOTOR_ID = 22;
    }

    public static class LoaderConstants {
        public static final int LOADER_MOTOR_ID = 31;
    }

    public static class ShooterConstants {
        public static final int SHOOTER_MOTOR_ID = 41;
        public static final String SHOOTER_LOG_KEY = "Subsystems/Shooter/";
        public static final double SHOOTER_HEIGHT_M = 0.7; //TODO: this is a placeholder value, get the actual height of the shooter and the angle of the shooter

    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 51;
    }

    public static class DrivetrainConstants {
        public static final double DISTANCE_TO_HUB_M = 2.5;//TODO: this is a placeholder value, need to calculate the actual value when we have a drivetrain
        public static final double WHEEL_COF = 1.1;
        public static final double MAX_DRIVE_SPEED = 5; //TODO: this needs to be tested
        public static final double MAX_ROTATIONAL_SPEED = 1; //TODO: this needs to be tested
        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(2, MAX_DRIVE_SPEED, WHEEL_COF, DCMotor.getKrakenX60Foc(4), 80, 4);
    }
}
