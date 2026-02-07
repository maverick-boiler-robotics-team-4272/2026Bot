package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class LoadAndShootCommand extends ParallelCommandGroup {

  public LoadAndShootCommand(Loader loader, Shooter shooter, BooleanSupplier readyToShoot, DoubleSupplier shooterRevSpeed, DoubleSupplier loaderSpeed) {
    super(
      Commands.run(shooter.rev(shooterRevSpeed),shooter),
      Commands.run(loader.load(loaderSpeed), loader)
        .beforeStarting(
          new WaitUntilCommand(readyToShoot)
        ),
        new ConditionalCommand(
          new InstantCommand(() -> {}), 
          Commands.run(loader.load(0)),
          readyToShoot)
    ); 
  }
}
