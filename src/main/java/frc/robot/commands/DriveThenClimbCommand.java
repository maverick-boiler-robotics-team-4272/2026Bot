package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.ClimberConstants.CLIMB_ROTATIONS;
import static frc.robot.constants.SubsystemConstants.ClimberConstants.UNLATCH_ROTATIONS;
import static frc.robot.subsystems.CommandSwerveDrivetrain.ROBOT_POSE;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveThenClimbCommand extends SequentialCommandGroup{
    public DriveThenClimbCommand(CommandSwerveDrivetrain drive, Climber climber) {
        super(
            new ParallelCommandGroup(
                drive.pidToPoint(ROBOT_POSE.nearest(CLIMB_POSES)),
                climber.climb(UNLATCH_ROTATIONS) // TODO: I don't know how many rotations this is
            ),
            new WaitUntilCommand(2),//maybe based on pose???
            climber.climb(CLIMB_ROTATIONS)
        );
    }
}
