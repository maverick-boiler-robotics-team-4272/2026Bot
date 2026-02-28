// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.FieldConstants.TUNE_POSE;
import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.MAX_DRIVE_SPEED;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.IDLE_SPEED;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.SHOOTER_LOG_KEY;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private ShuffleboardTab autoTab;
  private SendableChooser<Command> autoChooser;

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MAX_DRIVE_SPEED);

  public RobotContainer() {
    setDefaultCommands();
    configureBindings();

    setupAutos();
    intiElastic();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain.joystickDrive(joystick::getLeftX, joystick::getLeftY, joystick::getRightX));

    climber.setDefaultCommand(climber.climb(0));
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
    joystick.leftTrigger().whileTrue(intake.setIntakeState(EXTEND_DISTANCE, 45));// checkmark
    joystick.rightTrigger().whileTrue(hopper.agitate(42, 30));

    joystick.leftBumper().whileTrue(intake.setIntakeState(0, 0));

    joystick.a().whileTrue(
        ShooterCommands.fullShooterCommand(shooter, hopper, loader, intake,
            drivetrain, joystick::getLeftX,
            joystick::getLeftY, () -> joystick.getHID().getXButton())); // IT WORKS!!!!

    // joystick.povDown()
    // .whileTrue(
    // ClimbCommands.driveThenClimbCommand(drivetrain, climber, intake, () ->
    // joystick.getHID().getXButton())
    // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); // Yup

    operator.leftBumper().whileTrue(
        intake.agitateIntake());

    joystick.y().whileTrue(drivetrain.applyRequest(() -> brake));// sure

    joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));// not me

    joystick.leftStick().whileTrue(drivetrain.pidToPoint(TUNE_POSE));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // testing commands
    operator.b().whileTrue(shooter.setShooterState(0.08, 70));
    operator.a().whileTrue(shooter.setShooterState(() -> {
      return SmartDashboard.getNumber(SHOOTER_LOG_KEY + "angle", 0);
    }, () -> {
      return SmartDashboard.getNumber(SHOOTER_LOG_KEY + "speed", 0);
    }));

    operator.rightBumper().whileTrue(shooter.setVoltage(() -> {
      return SmartDashboard.getNumber(SHOOTER_LOG_KEY + "angle", 0);
    }, () -> {
      return SmartDashboard.getNumber(SHOOTER_LOG_KEY + "speed", 0);
    }));

    operator.x().whileTrue(loader.loadBoth(30));

    operator.y().whileTrue(hopper.agitate(42, 25));
    operator.povLeft().whileTrue(intake.zeroExtension());
    operator.povRight().whileTrue(shooter.zeroHood());

    operator.leftTrigger().whileTrue(
        drivetrain.pidThroughTrench());
    operator.rightTrigger().whileTrue(new PathPlannerAuto("Test",false));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void registerNamedCommands() {
    // NamedCommands.registerCommand("EXAMPLE", command);
    NamedCommands.registerCommand(
        "Intake",
        intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED));
  }

  public void setupAutos() {
    autoChooser = new SendableChooser<>();

    autoTab = Shuffleboard.getTab("Auto");
    autoTab.add("AutoChooser", autoChooser);

    // autoChooser.addOption("Example", new PathPlannerAuto("EXACT NAME OF THE
    // PATHPLANNER AUTO", /*mirror*/false))
    autoChooser.setDefaultOption("New Auto", new PathPlannerAuto("New Auto", false));
  }

  public void intiElastic() {
    SmartDashboard.putNumber(SHOOTER_LOG_KEY + "angle", 0);
    SmartDashboard.putNumber(SHOOTER_LOG_KEY + "speed", 50);
  }

  public Command getAutonomousCommand() {
    // var auto = new PathPlannerAuto("New Auto", false).withName("auto command");
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromChoreoTrajectory("test");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    var auto = AutoBuilder.followPath(path);
    return auto;
  }
}
