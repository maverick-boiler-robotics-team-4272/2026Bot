package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
    public static Command teleHalfShooterCommand(
            Shooter shooter,
            Hopper hopper,
            CommandSwerveDrivetrain drive,
            DoubleSupplier joystickX,
            DoubleSupplier joystickY) {
        return new ParallelCommandGroup(
                hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
                drive.pointTowardsPoint(HUB_LOCATION, joystickX, joystickY),
                setDesiredShooterStates(shooter, drive)

        );
    }

    public static Command tele2ndHalfShooterCommand(
            Loader loader, Intake intake) {
        return new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                        loader.loadBoth(70),
                        intake.agitateIntake()));
    }

    public static Command autoShooCommand(
            Shooter shooter, Hopper hopper, Loader loader, Intake intake) {
        return new ParallelCommandGroup(
                hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
                shooter.setShooterState(AUTO_SHOOTER_HOOD, AUTO_SHOOTER_VELOCITY),
                Commands.repeatingSequence(
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        intake.agitateIntake(),
                                        loader.loadBoth(70)),
                                loader.loadBoth(0), shooter::isAtDesiredSpeed)));
    }

    public static Command setDesiredShooterStates(Shooter shooter, CommandSwerveDrivetrain drive) {
        return shooter.defer(() -> Commands.repeatingSequence(
                shooter.setShooterState(
                        () -> SCORE_ANGLE_LOOKUP.get(drive.getState().Pose.getTranslation()
                                .getDistance(HUB_LOCATION)),
                        () -> SHOOTER_VELOCITY_LOOKUP
                                .get(drive.getState().Pose.getTranslation()
                                        .getDistance(HUB_LOCATION)))
                        .until(() -> {
                            return !(isRedSide()
                                    ? drive.getState().Pose.getX() > FIELD_LENGTH_M
                                            - 4.03
                                    : drive.getState().Pose.getX() < 4.03);
                        }), // (isRedSide ? drive.getState().Pose.getX() > FIELD_LENGTH_M - 4.03
                            // :
                            // drive.getState().Pose.getX() < 4.03)
                shooter.setShooterState(40, 45).until(
                        () -> {
                            return (isRedSide()
                                    ? drive.getState().Pose.getX() > FIELD_LENGTH_M
                                            - 4.03
                                    : drive.getState().Pose.getX() < 4.03);
                        })));
    }
}
