package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;


import com.ctre.phoenix6.swerve.SwerveRequest;

public class ShooterCommands {
        public static Command teleHalfShooterCommand(
                        Shooter shooter,
                        CommandSwerveDrivetrain drive,
                        DoubleSupplier joystickX,
                        DoubleSupplier joystickY) {
                return new ParallelCommandGroup(
                                drive.defer(() -> Commands.repeatingSequence(
                                new ConditionalCommand(drive.pointTowardsPoint(
                                        () -> getHubLocation().getTranslation(),
                                                                joystickX,
                                        joystickY),
                                        drive.pointTowardsPoint(
                                                () -> drive.getState().Pose.nearest(
                                                        getShuttlePoses())
                                                        .getTranslation(),
                                                joystickX, joystickY)
                                                .until(drive.isInAllianceZone()),
                                        drive.isInAllianceZone())

                        ))
                                .until(drive::isAtDesiredAngle)
                                .andThen(drive.defer(() -> drive.applyRequest(
                                        () -> new SwerveRequest.SwerveDriveBrake())))
                                .until(() -> !drive.isAtDesiredAngle() || joystickX.getAsDouble() != 0.0 || joystickY.getAsDouble() != 0.0).repeatedly(),
                        setDesiredShooterStates(shooter, drive));
        }

        public static Command tele2ndHalfShooterCommand(
                        Loader loader, Intake intake, Hopper hopper, Shooter shooter, CommandSwerveDrivetrain drive) {
                return new SequentialCommandGroup(
                                // new WaitUntilCommand(shooter::isAtDesiredSpeed),
                        // new WaitUntilCommand(shooter::isAtDesiredAngle),
                        new ParallelCommandGroup(
                                loader.loadBoth(-50),
                                hopper.agitate(-HOPPER_LOWER_SPEED * 2, -HOPPER_UPPER_SPEED)
                                )
                                .withTimeout(0.2).onlyWhile(() -> !shooter.isAtDesiredSpeed()),
                                Commands.repeatingSequence(
                                new ParallelCommandGroup(
                                        loader.loadBoth(50),
                                        intake.agitateIntake(),
                                        hopper.agitate(HOPPER_LOWER_SPEED,
                                                HOPPER_UPPER_SPEED))
                                        .unless(() -> {
                                            return drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135)
                                                    && drive.isNotInAllianceZone().getAsBoolean();
                                        }))
                                .repeatedly());
        }

        public static Command tele2ndHalfShooterCommandWithIntake(Loader loader, Intake intake, Hopper hopper, Shooter shooter, CommandSwerveDrivetrain drive) {
            return new SequentialCommandGroup(
                new WaitUntilCommand(0.15),
                // new WaitUntilCommand(0.2),
                Commands.repeatingSequence(
                new ParallelCommandGroup(
                                loader.loadBoth(70),
                                intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED),
                                                hopper.agitate(HOPPER_LOWER_SPEED,
                                                                HOPPER_UPPER_SPEED))
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
                                    () -> {
                                        return SCORE_ANGLE_LOOKUP_FAR
                                                                        .get(
                                                                                drive.getState().Pose
                                                                                        .getTranslation()
                                                                                        .getDistance(getHubLocation()
                                                                        .getTranslation()));
                                    },
                                    () -> {
                                        return SHOOTER_VELOCITY_LOOKUP.get(
                                                                        drive.getState().Pose
                                                                                .getTranslation()
                                                                                .getDistance(getHubLocation()
                                                                .getTranslation())) + 2;
                                    })
                                    .until(drive.isNotInAllianceZone()),
                            shooter.defer(
                                    () -> Commands.repeatingSequence(shooter.setShooterState(
                                            () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                    && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135) ? 0 : 0.08,
                                            () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                    && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135)
                                                                    ? 0
                                                                    : SHUTTLE_SPEED_TABLE
                                                                            .get(drive.getState().Pose.getTranslation()
                                                                                    .getDistance(drive.getState().Pose
                                                                                            .nearest(getShuttlePoses())
                                                                                            .getTranslation())))))
                                    .until(() -> drive.isInAllianceZone().getAsBoolean()
                                            || drive.isInOpposingAllianceZone()
                                                    .getAsBoolean()),
                            shooter.defer(
                                    () -> Commands.repeatingSequence(
                                            shooter.setShooterState(
                                                    () -> drive.getState().Pose
                                                            .getY() > Units.inchesToMeters(
                                                                    135)
                                                            && drive.getState().Pose
                                                                    .getY() < FIELD_WIDTH_M
                                                                            - Units
                                                                                    .inchesToMeters(135)
                                                                                            ? 0
                                                                                            : 0.075,
                                                    () -> drive.getState().Pose
                                                            .getY() > Units.inchesToMeters(
                                                                    135)
                                                            && drive.getState().Pose
                                                                    .getY() < FIELD_WIDTH_M
                                                                            - Units
                                                                                    .inchesToMeters(135)
                                                                                            ? 0
                                                                                            : 85)))
                                    .until(() -> !drive.isInOpposingAllianceZone()
                                            .getAsBoolean()))));
        }
}
