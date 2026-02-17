package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends ParallelCommandGroup{
    public ShootCommand(Shooter shooter, Loader loader) {
        super(
            shooter.setDesiredSpeed(),
            shooter.setDesiredAngle(),
            new WaitUntilCommand(shooter::isAtDesiredSpeed),
            loader.loadBoth(70)
        );
    }
}
