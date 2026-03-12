package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class ShooterCommands {
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
                        Loader loader, Intake intake, Hopper hopper) {
                return new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new ParallelCommandGroup(
                                                loader.loadBoth(70),
                                                intake.agitateIntake(),
                                                hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED)).repeatedly());
        }

        public static Command setDesiredShooterStates(Shooter shooter, CommandSwerveDrivetrain drive) {
            return new ParallelCommandGroup(
                    shooter.defer(() -> Commands.repeatingSequence(
                            shooter.setShooterState(
                                    () -> SCORE_ANGLE_LOOKUP.get(drive.getState().Pose
                                            .getTranslation()
                                            .getDistance(getHubLocation()
                                                    .getTranslation())),
                                    () -> SHOOTER_VELOCITY_LOOKUP
                                            .get(drive.getState().Pose.getTranslation()
                                                    .getDistance(getHubLocation()
                                                            .getTranslation())))
                                    .until(drive.isNotInAllianceZone()),
                            shooter.defer(
                                    () -> Commands.repeatingSequence(shooter.setShooterState(
                                            () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                    && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135) ? 0 : 0.7,
                                            () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                                                    && drive.getState().Pose.getY() < FIELD_WIDTH_M - Units
                                                            .inchesToMeters(135) ? 0 : 70))
                            // () -> Commands.repeatingSequence(new ConditionalCommand(
                            // shooter.setShooterState(
                            // 0, 0),
                            // shooter.setShooterState(0.7, 70),
                            // () -> drive.getState().Pose.getY() > Units.inchesToMeters(135)
                            // && drive.getState().Pose.getY() < FIELD_LENGTH_M - Units
                            // .inchesToMeters(135)))
                            )
                                    .until(drive.isInAllianceZone()))));
        }
}
