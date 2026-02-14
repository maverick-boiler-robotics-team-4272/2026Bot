package frc.robot.commands;

import static frc.robot.constants.FieldConstants.HUB_LOCATION;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class ArmShooterCommand extends ParallelCommandGroup {
    public ArmShooterCommand(Shooter shooter, Hopper hopper, CommandSwerveDrivetrain drive, DoubleSupplier joystickX, DoubleSupplier joystickY) {
        super(
            hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
            shooter.setDesiredAngle(),
            drive.pointTowardsPoint(HUB_LOCATION, joystickX, joystickY)
        );
    }
}
