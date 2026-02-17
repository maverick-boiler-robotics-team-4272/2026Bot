package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

public class IntakeCommands {
  public static Command intakeCommand(Intake intake, Hopper hopper) {
    return new ParallelCommandGroup(
        intake.extend(EXTEND_DISTANCE),
        intake.intake(INTAKE_SPEED),
        hopper.agitate(HOPPER_UPPER_SPEED, HOPPER_UPPER_SPEED));
  }
}