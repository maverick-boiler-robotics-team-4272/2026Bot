// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.MAX_DRIVE_SPEED;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.TunerConstants;
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
  // private final Climber climber = new Climber();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private SendableChooser<Command> autoChooser;

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MAX_DRIVE_SPEED);

  public RobotContainer() {
    setDefaultCommands();
    configureBindings();

    setupAutos();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain.joystickDrive(joystick::getLeftX, joystick::getLeftY, () -> {
          return Math.pow(joystick.getRightX(), 3);
        }));

    // climber.setDefaultCommand(climber.climb(0));
    hopper.setDefaultCommand(hopper.stop());
    intake.setDefaultCommand(intake.setDefaultCommand());
    loader.setDefaultCommand(loader.loadBoth(0));
    shooter.setDefaultCommand(shooter.defaultCommand());

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("Idle"));
  }

  private void configureBindings() {
    joystick.leftTrigger().whileTrue(intake.setIntakeState(EXTEND_DISTANCE, 50));// checkmark
    joystick.rightTrigger().whileTrue(intake.setIntakeState(0, 0));

    joystick.leftBumper().whileTrue(drivetrain.doTrenches());

    joystick.a().whileTrue(
        ShooterCommands.teleHalfShooterCommand(shooter, hopper,
            drivetrain, joystick::getLeftX,
            joystick::getLeftY)); // IT WORKS!!!!
    joystick.a().whileTrue(
        ShooterCommands.tele2ndHalfShooterCommand(loader, intake));

    joystick.povLeft().whileTrue(
        intake.outZeroExtension());

    joystick.povRight().whileTrue(
        shooter.zeroHood());

    // joystick.povDown()
    // .whileTrue(
    // ClimbCommands.driveThenClimbCommand(drivetrain, climber, intake, () ->
    // joystick.getHID().getXButton())
    // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); // Yup

    joystick.y().whileTrue(drivetrain.applyRequest(() -> brake));// sure

    joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));// not me

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // testing commands
    operator.povLeft().whileTrue(intake.zeroExtension());
    operator.povRight().whileTrue(shooter.zeroHood());

    operator.leftTrigger().whileTrue(
        hopper.agitate(50, 10));

    operator.rightTrigger().whileTrue(
        intake.agitateIntake());

    operator.a().whileTrue(
        shooter.setShooterState(0.015, 50));
    // operator.rightTrigger().whileTrue(new PathPlannerAuto("Test", false));

    drivetrain.registerTelemetry(logger::telemeterize);

    // operator.a().whileTrue(shooter.setShooterState(() ->
    // SmartDashboard.getNumber("ANGLE", 0),
    // () -> SmartDashboard.getNumber("SPEED", 0)));
    // operator.x().whileTrue(new ParallelCommandGroup(
    // intake.agitateIntake(),
    // hopper.agitate(50, 10),
    // loader.loadBoth(70)).repeatedly());
  }

  public void registerNamedCommands() {
    // NamedCommands.registerCommand("EXAMPLE", command);
    NamedCommands.registerCommand(
        "Intake",
        intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED));
  }

  public void setupAutos() {
    autoChooser = new SendableChooser<>();
    SmartDashboard.putData("Auto chooser", autoChooser);

    // PathPlannerPath ExamplePath;
    // try {
    // ExamplePath = PathPlannerPath.fromChoreoTrajectory("Exact Name of Path");
    // } catch (Exception e) {
    // throw new RuntimeException("Failed to load Choreo trajectory: " +
    // e.getMessage());
    // }

    PathPlannerPath startRight;
    try {
      startRight = PathPlannerPath.fromChoreoTrajectory("Start_To_Mid_Right");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath intakeRight;
    try {
      intakeRight = PathPlannerPath.fromChoreoTrajectory("Mid_Right_Intake");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath midRightShoot;
    try {
      midRightShoot = PathPlannerPath.fromChoreoTrajectory("Mid_To_Right_Shoot");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath RightShootToOutpost;
    try {
      RightShootToOutpost = PathPlannerPath.fromChoreoTrajectory("Right_Shoot_To_Outpost");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath OutpostToRightShoot;
    try {
      OutpostToRightShoot = PathPlannerPath.fromChoreoTrajectory("Outpost_To_Right_Shoot");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    // Command exampleAuto = new SequentialCommandGroup(
    // AutoBuilder.followPath(startRight),
    // new ParallelCommandGroup(
    // AutoBuilder.followPath(intakeRight),
    // intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED).withTimeout(1.5)
    // ),
    // );

    Command rightSideOneAuto = new SequentialCommandGroup(

        AutoBuilder.followPath(startRight),
        new ParallelCommandGroup(
            AutoBuilder.followPath(intakeRight),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED).withTimeout(1.5)),
        AutoBuilder.followPath(midRightShoot),
        shooter.setShooterState(.05, 50).withTimeout(5));

    Command rightSideOneAndOutpostAuto = new SequentialCommandGroup(
        AutoBuilder.followPath(startRight),
        new ParallelCommandGroup(
            AutoBuilder.followPath(intakeRight),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED).withTimeout(1.5)),
        AutoBuilder.followPath(midRightShoot),
        shooter.setShooterState(.05, 50).withTimeout(5),
        AutoBuilder.followPath(RightShootToOutpost),
        shooter.setShooterState(EXTEND_DISTANCE, INTAKE_SPEED).withTimeout(3),
        AutoBuilder.followPath(OutpostToRightShoot),
        shooter.setShooterState(EXTEND_DISTANCE, INTAKE_SPEED).withTimeout(10));

    // autoChooser.setDefaultOption("Example", exampleAuto);
    autoChooser.setDefaultOption("Right one Cycle", rightSideOneAuto);
    autoChooser.addOption("Right one and Outpost Cycle", rightSideOneAndOutpostAuto);

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
