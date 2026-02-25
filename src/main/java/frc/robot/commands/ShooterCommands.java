package frc.robot.commands;

import static frc.robot.constants.FieldConstants.FIELD_LENGTH_M;
import static frc.robot.constants.FieldConstants.HUB_LOCATION;
import static frc.robot.constants.FieldConstants.isRedSide;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  public static Command fullShooterCommand(
      Shooter shooter,
      Hopper hopper,
      Loader loader,
      Intake intake,
      CommandSwerveDrivetrain drive,
      DoubleSupplier joystickX,
      DoubleSupplier joystickY,
      BooleanSupplier isReady) {
    return new ParallelCommandGroup(
        hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
        intake.agitateIntake(),
        drive.pointTowardsPoint(HUB_LOCATION, joystickX, joystickY),
        setDesiredShooterStates(shooter, drive),
        Commands.repeatingSequence(
            new ConditionalCommand(
                loader.loadBoth(70),
                loader.loadBoth(0),
                () -> {
                  return isReady.getAsBoolean() && shooter.isAtDesiredSpeed();
                })));
  }

  public static Command setDesiredShooterStates(Shooter shooter, CommandSwerveDrivetrain drive) {
    return shooter.defer(() -> Commands.repeatingSequence(
        shooter.setShooterState(
            () -> SCORE_ANGLE_LOOKUP.get(drive.getState().Pose.getTranslation().getDistance(HUB_LOCATION)),
            () -> SHOOTER_VELOCITY)
            .until(() -> {
              return !(isRedSide() ? drive.getState().Pose.getX() > FIELD_LENGTH_M - 4.03
                  : drive.getState().Pose.getX() < 4.03);
            }), // (isRedSide ? drive.getState().Pose.getX() > FIELD_LENGTH_M - 4.03 :
                // drive.getState().Pose.getX() < 4.03)
        shooter.setShooterState(40, 45).until(
            () -> {
              return (isRedSide() ? drive.getState().Pose.getX() > FIELD_LENGTH_M - 4.03
                  : drive.getState().Pose.getX() < 4.03);
            })));
  }
}
