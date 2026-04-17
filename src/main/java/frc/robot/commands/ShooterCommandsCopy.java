package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ShooterCommandsCopy{
        private static Translation2d currentLocation;
        public static Command teleHalfShooterCommand(
                        Shooter shooter,
                        CommandSwerveDrivetrain drive,
                        DoubleSupplier joystickX,
                        DoubleSupplier joystickY) {
                return new ParallelCommandGroup(
                                drive.defer(() -> Commands.repeatingSequence(
                                                drive.pointTowardsPoint(() -> {
                                                    currentLocation = getHubLocation().getTranslation();
                                                    Translation2d robotPos = drive.getState().Pose.getTranslation();
                                                    double vx = drive.getVelocityX().getAsDouble();
                                                    double vy = drive.getVelocityY().getAsDouble();
                                                    for (int i = 0; i < 15; i++) {
                                                        double tof = TufF_TABLE.get(robotPos.getDistance(currentLocation));
                                                        currentLocation = getHubLocation().getTranslation().minus(new Translation2d(vx * tof, vy * tof));
                                                        DogLog.log("Subsystems/Drive/Virtual Hub", new Pose2d(currentLocation, Rotation2d.kZero));
                                                    }

                                                        return currentLocation;},
                                                                () -> joystickX.getAsDouble() / 3,
                                                                () -> joystickY.getAsDouble() / 3)
                                        .until(drive.isNotInAllianceZone()),
                                                drive.pointTowardsPoint(
                                                                () -> drive.getState().Pose.nearest(getShuttlePoses())
                                                                                .getTranslation(),
                                                                joystickX, joystickY)
                                        .until(drive.isInAllianceZone()))),
                                setDesiredShooterStates(shooter, drive));
        }

        public static Command tele2ndHalfShooterCommand(
                        Loader loader, Intake intake, Hopper hopper, Shooter shooter, CommandSwerveDrivetrain drive) {
                return new SequentialCommandGroup(
                                // new WaitUntilCommand(shooter::isAtDesiredSpeed),
                                // new WaitUntilCommand(shooter::isAtDesiredAngle),
                                new ParallelCommandGroup(
                                        loader.loadBoth(-50),
                                        hopper.agitate(-HOPPER_LOWER_SPEED*2, -HOPPER_UPPER_SPEED)
                                ).withTimeout(0.15).onlyWhile(() -> !shooter.isAtDesiredSpeed()),
                                Commands.repeatingSequence(
                                new ParallelCommandGroup(
                                                loader.loadBoth(70),
                                                intake.agitateIntake(),
                                        hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED))
                                        .unless(() -> {
                                            return drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135)
                                                    && drive.isNotInAllianceZone().getAsBoolean();
                                        }))
                                .repeatedly());
        }

        public static Command tele2ndHalfShooterCommandWithIntake(
                Loader loader, Intake intake, Hopper hopper, Shooter shooter, CommandSwerveDrivetrain drive) {
        return new SequentialCommandGroup(
                        // new WaitUntilCommand(shooter::isAtDesiredSpeed),
                        // new WaitUntilCommand(shooter::isAtDesiredAngle),
                        new ParallelCommandGroup(
                                        loader.loadBoth(-50),
                                        hopper.agitate(-HOPPER_LOWER_SPEED*2, -HOPPER_UPPER_SPEED)
                                ).withTimeout(0.15).onlyWhile(() -> !shooter.isAtDesiredSpeed()),
                        Commands.repeatingSequence(
                        new ParallelCommandGroup(
                                        loader.loadBoth(70),
                                        intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED),
                                hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED))
                                .unless(() -> {
                                    return drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                        && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                    .inchesToMeters(135)
                                            && drive.isNotInAllianceZone().getAsBoolean();
                                }))
                        .repeatedly());
}

        public static Command setDesiredShooterStates(Shooter shooter, CommandSwerveDrivetrain drive) {
            return new ParallelCommandGroup(
                    shooter.defer(() -> Commands.repeatingSequence(
                            shooter.setShooterState(
                                    () -> SCORE_ANGLE_LOOKUP_FAR.get(drive.getState().Pose
                                            .getTranslation()
                                            .getDistance(currentLocation)),
                                    () -> SHOOTER_VELOCITY_LOOKUP
                                            .get(drive.getState().Pose.getTranslation()
                                                    .getDistance(currentLocation)))
                                    .until(drive.isNotInAllianceZone()),
                            shooter.defer(
                                    () -> Commands.repeatingSequence(shooter.setShooterState(
                                            () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                    && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135) ? 0 : 0.7,
                                            () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                    && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135) ? 0 : 70)))
                                    .until(drive.isInAllianceZone())))
                                    );
        }

}
