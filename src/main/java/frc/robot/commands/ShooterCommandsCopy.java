package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_LOWER_SPEED;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_UPPER_SPEED;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
                                                    for (int i = 0; i < 10; i++) {
                                                        double tof = TufF_TABLE.get(robotPos.getDistance(currentLocation));
                                                        currentLocation = getHubLocation().getTranslation().minus(new Translation2d(vx * tof, vy * tof));
                                                        DogLog.log("Subsystems/Drive/Virtual Hub", new Pose2d(currentLocation, Rotation2d.kZero));
                                                    }

                                                        return currentLocation;},
                                                                joystickX,
                                                                joystickY)
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
                                new WaitUntilCommand(shooter::isAtDesiredSpeed),
                                new WaitUntilCommand(shooter::isAtDesiredAngle),
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
