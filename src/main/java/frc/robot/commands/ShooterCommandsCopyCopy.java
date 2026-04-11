package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.FieldConstants.getHubLocation;
import static frc.robot.constants.FieldConstants.getShuttlePoses;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_LOWER_SPEED;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_UPPER_SPEED;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;


import com.ctre.phoenix6.swerve.SwerveRequest;

public class ShooterCommandsCopyCopy
 {
    public static double shooterAdd = 0;
    public static double angleAdd = 0;
        public static Command teleHalfShooterCommand(
                        Shooter shooter,
                        CommandSwerveDrivetrain drive,
                        DoubleSupplier joystickX,
                        DoubleSupplier joystickY) {
                return new ParallelCommandGroup(
                                drive.defer(() -> Commands.repeatingSequence(
                                                drive.pointTowardsPoint(() -> getHubLocation().getTranslation(),
                                                                joystickX,
                                                                joystickY)
                                        .until(drive.isNotInAllianceZone()),
                                                drive.pointTowardsPoint(
                                                                () -> drive.getState().Pose.nearest(getShuttlePoses())
                                                                                .getTranslation(),
                                                                joystickX, joystickY)
                                        .until(drive.isInAllianceZone()))),
                        setDesiredShooterStates(shooter, drive))
                        .andThen(drive.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
        }

        public static Command tele2ndHalfShooterCommand(
                        Loader loader, Intake intake, Hopper hopper, Shooter shooter, CommandSwerveDrivetrain drive) {
                return new SequentialCommandGroup(
                                new WaitUntilCommand(0.2),
                                Commands.repeatingSequence(
                                new ParallelCommandGroup(
                                                loader.loadBoth(70),
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
                new WaitUntilCommand(0.2),
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
                                                                                        .getTranslation())) + getShooterAdd().getAsDouble();
                                    })
                                    .until(drive.isNotInAllianceZone()),
                            shooter.defer(
                                    () -> Commands.repeatingSequence(shooter.setShooterState(
                                            () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                    && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135) ? 0 : 0.07,
                                            () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                    && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135) ? 0 : 70)))
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

        public static Command addShooterAdd(Shooter shooter) {
            return Commands.runOnce(() -> shooterAdd += 0.5, shooter);
        }

        public static Command subtractShooterAdd(Shooter shooter) {
            return Commands.runOnce(() -> shooterAdd -= 0.5, shooter);
        }

        public static DoubleSupplier getShooterAdd() {
            return () -> shooterAdd;
        }

        public static Command addAngleAdd(Shooter shooter) {
            return Commands.runOnce(() -> angleAdd += 0.005, shooter);
        }

        public static Command subtractAngleAdd(Shooter shooter) {
            return Commands.runOnce(() -> angleAdd -= 0.005, shooter);
        }
}
