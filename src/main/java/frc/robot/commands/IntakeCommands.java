package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
    public static Command intakeCommand(Intake intake, CommandSwerveDrivetrain drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new ParallelCommandGroup(
            new ConditionalCommand(
                new ParallelCommandGroup(
                    intake.driverIntake(),
                    drive.joystickDrive(() -> x.getAsDouble() / 3, () -> y.getAsDouble() / 3, () -> theta.getAsDouble() / 3)
                ), 
                new ParallelCommandGroup(
                    intake.driverIntake(),
                    drive.joystickDrive(x, y, theta)
                ),
                drive.isInOpposingAllianceZone())
        ).repeatedly();
    }
}
