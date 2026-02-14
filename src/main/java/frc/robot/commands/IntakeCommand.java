package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

public class IntakeCommand extends ParallelCommandGroup {
    
  public IntakeCommand(Intake intake, Hopper hopper){
    super(
        intake.extend(EXTEND_DISTANCE),
        
        intake.intake(INTAKE_SPEED),
        hopper.agitate(HOPPER_UPPER_SPEED, HOPPER_UPPER_SPEED)
    );
  }
}