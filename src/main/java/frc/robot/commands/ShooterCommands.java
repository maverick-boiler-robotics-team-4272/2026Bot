package frc.robot.commands;

import static frc.robot.constants.FieldConstants.HUB_LOCATION;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  public static Command armShooterCommand(
      Shooter shooter,
      Hopper hopper,
      CommandSwerveDrivetrain drive,
      DoubleSupplier joystickX,
      DoubleSupplier joystickY) {
    return new ParallelCommandGroup(
        hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
        shooter.setDesiredAngle(),
        drive.pointTowardsPoint(HUB_LOCATION, joystickX, joystickY));
  }

  public static Command shootCommand(Shooter shooter, Loader loader) {
    return new ParallelCommandGroup(
        shooter.setDesiredSpeed(),
        new WaitUntilCommand(shooter::isAtDesiredSpeed),
        loader.loadBoth(70));
  }
}
