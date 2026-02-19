package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.ClimberConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ClimbCommands {
  public static Command driveThenClimbCommand(CommandSwerveDrivetrain drive, Climber climber, BooleanSupplier ready) {
    return new ParallelCommandGroup(
        drive.defer(() -> drive.pidToPoint(drive.getState().Pose.nearest(CLIMB_POSES))),
        Commands.repeatingSequence(
            new SequentialCommandGroup(
                climber.climb(UNLATCH_ROTATIONS).until(ready),
                climber.climb(CLIMB_ROTATIONS))));
  }
}
