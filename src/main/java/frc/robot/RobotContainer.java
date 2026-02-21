// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.MAX_DRIVE_SPEED;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.IDLE_SPEED;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final Hopper hopper = new Hopper();
  private final Intake intake = new Intake();
  private final Loader loader = new Loader();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MAX_DRIVE_SPEED);

  public RobotContainer() {
    setDefaultCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain.joystickDrive(joystick::getLeftX, joystick::getLeftY, joystick::getRightX));

    climber.setDefaultCommand(climber.climb(0));
    hopper.setDefaultCommand(hopper.agitate(0, 0));
    intake.setDefaultCommand(intake.defaultCommand());
    loader.setDefaultCommand(loader.loadBoth(0));
    shooter.setDefaultCommand(shooter.rev(IDLE_SPEED));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
  }

  private void configureBindings() {
    joystick.leftTrigger().whileTrue(intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED));// checkmark
    joystick.leftTrigger().whileTrue(hopper.agitate(10, 5));

    joystick.leftBumper().whileTrue(intake.setIntakeState(0, 0));

    joystick.a().whileTrue(
        ShooterCommands.fullShooterCommand(shooter, hopper, loader, drivetrain, joystick::getLeftX,
            joystick::getLeftY, () -> joystick.getHID().getXButton())); // IT WORKS!!!!

    joystick.povDown()
        .whileTrue(
            ClimbCommands.driveThenClimbCommand(drivetrain, climber, intake, () -> joystick.getHID().getXButton())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); // Yup

    joystick.y().whileTrue(drivetrain.applyRequest(() -> brake));// sure

    joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));// not me

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // testing commands
    operator.rightBumper().whileTrue(intake.extend(10.6));
    operator.leftBumper().whileTrue(intake.extend(0.0));
    operator.a().whileTrue(shooter.setShooterState(0.04, 50));
    operator.b().whileTrue(shooter.setShooterState(0.08, 50));

    operator.x().whileTrue(loader.loadBoth(20));
    operator.y().whileTrue(hopper.agitate(20, 20));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No Auto");
  }
}
