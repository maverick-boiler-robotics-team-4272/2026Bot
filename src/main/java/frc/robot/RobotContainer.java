// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.FieldConstants.isRedSide;
import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.MAX_DRIVE_SPEED;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_LOWER_SPEED;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_UPPER_SPEED;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.constants.FieldConstants.*;

import java.lang.reflect.Field;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommandsCopy;
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

  public static final CommandXboxController joystick = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MAX_DRIVE_SPEED);

  public RobotContainer() {
    setDefaultCommands();
    configureBindings();

    setupAutos();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain.joystickDrive(joystick::getLeftX, joystick::getLeftY, joystick::getRightX));

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
    joystick.leftTrigger().whileTrue(intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED));// checkmark
    joystick.leftBumper().whileTrue(intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED));// checkmark
    joystick.rightTrigger().whileTrue(intake.barf());

    // joystick.leftBumper().whileTrue(drivetrain.doTrenches());

    // shoot rev
    joystick.a().whileTrue(
        ShooterCommands.teleHalfShooterCommand(shooter,
            drivetrain, joystick::getLeftX,
            joystick::getLeftY).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); // IT WORKS!!!!
    // shoot with intake agitation
    joystick.a().whileTrue( 
        ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain));
    // shoot and intake fuel
    joystick.a().and(joystick.leftTrigger()).whileTrue(
        ShooterCommands.tele2ndHalfShooterCommandWithIntake(loader, intake, hopper, shooter, drivetrain));

    joystick.povLeft().whileTrue(
        intake.outZeroExtension());

    joystick.povRight().whileTrue(
        shooter.zeroHood()); 
    joystick.rightBumper().whileTrue(ShooterCommandsCopy.teleHalfShooterCommand(shooter, drivetrain, joystick::getLeftX, joystick::getLeftY));
    // joystick.y().whileTrue(ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain));

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

    operator.y().whileTrue(
        shooter.setShooterState(0.015, 50));
    // operator.rightTrigger().whileTrue(new PathPlannerAuto("Test", false));

    drivetrain.registerTelemetry(logger::telemeterize);

    // shuttle button since auto shuttling no work question mark
    operator.a().whileTrue(
        shooter.setShooterState(0.07, 60));

    // operator.x().whileTrue(
    // hopper.agitate(-50, -30));

    operator.b().whileTrue(shooter.setShooterState(() ->
    SmartDashboard.getNumber("ANGLE", 0.02),
    () -> SmartDashboard.getNumber("SPEED", 50)));
    
    operator.x().whileTrue(new ParallelCommandGroup(
        intake.agitateIntake(),
        hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
        loader.loadBoth(70)).repeatedly());
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

    SmartDashboard.putNumber("ANGLE", 0.02);
    SmartDashboard.putNumber("SPEED", 50);
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

    PathPlannerPath startLeft;
    try {
      startLeft = PathPlannerPath.fromChoreoTrajectory("Start_To_Mid_Left");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath intakeLeft;
    try {
      intakeLeft = PathPlannerPath.fromChoreoTrajectory("Mid_Left_Intake");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath midLeftShoot;
    try {
      midLeftShoot = PathPlannerPath.fromChoreoTrajectory("Mid_To_Left_Shoot");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath midRightOutpostShoot;
    try {
      midRightOutpostShoot = PathPlannerPath.fromChoreoTrajectory("Mid_To_OutPost_Shoot");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath rightOutpostShootToOutpost;
    try {
      rightOutpostShootToOutpost = PathPlannerPath.fromChoreoTrajectory("Outpost_Shoot_to_Outpost");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath outpostToOutpostShoot;
    try {
      outpostToOutpostShoot = PathPlannerPath.fromChoreoTrajectory("Outpost_to_Outpost_Shoot");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath MidleftToDepotShoot;
    try {
      MidleftToDepotShoot = PathPlannerPath.fromChoreoTrajectory("Left_Mid_To_Depot_Shoot");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath DepotShootToDepot;
    try {
      DepotShootToDepot = PathPlannerPath.fromChoreoTrajectory("Left_Shoot_To_Depot");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trjectory: " + e.getMessage());
    }

    PathPlannerPath PutItInReverseTerry;
    try {
      PutItInReverseTerry = PathPlannerPath.fromChoreoTrajectory("Put_It_In_Reverse_Terry");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath DepotToShoot;
    try {
      DepotToShoot = PathPlannerPath.fromChoreoTrajectory("Depot_To_Depot_Shoot");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath AirplaneTakeDownRight;
    try {
      AirplaneTakeDownRight = PathPlannerPath.fromChoreoTrajectory("Boeing_Door");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath AirplaneTakeDownLeft;
    try {
      AirplaneTakeDownLeft = PathPlannerPath.fromChoreoTrajectory("Boeing_Cockpit");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath RightControledChaos;
    try {
      RightControledChaos = PathPlannerPath.fromChoreoTrajectory("IDk_what_to_call_this");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory: " + e.getMessage());
    }

    PathPlannerPath newRightStart;
    try {
      newRightStart = PathPlannerPath.fromChoreoTrajectory("New_Right_Start");
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
        // drivetrain.resetThePose(new Pose2d(!isRedSide() ? 4.4 : FIELD_LENGTH_M - 4.4,
        // !isRedSide() ? 0.45 : FIELD_WIDTH_M - 0.45, !isRedSide() ?
        // Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg)),
        new ParallelRaceGroup(
            AutoBuilder.followPath(startRight),
            intake.outZeroExtension()),
        new ParallelRaceGroup(
            AutoBuilder.followPath(intakeRight),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
        AutoBuilder.followPath(midRightShoot),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain))));

    Command rightSideOneAndOutpostAuto = new SequentialCommandGroup(
        // drivetrain.resetThePose(new Pose2d(!isRedSide() ? 4.4 : FIELD_LENGTH_M - 4.4,
        // !isRedSide() ? 0.45 : FIELD_WIDTH_M - 0.45, !isRedSide() ?
        // Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg)),
        new ParallelRaceGroup(
            AutoBuilder.followPath(startRight),
            intake.outZeroExtension()),
        new ParallelRaceGroup(
            AutoBuilder.followPath(intakeRight),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
        AutoBuilder.followPath(midRightShoot),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain)))
            .withTimeout(5),
        AutoBuilder.followPath(RightShootToOutpost),
        intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED).withTimeout(3),
        AutoBuilder.followPath(OutpostToRightShoot),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain))));

    Command rightSideOneAndOutpostAutoDif = new SequentialCommandGroup(
        // drivetrain.resetThePose(new Pose2d(!isRedSide() ? 4.4 : FIELD_LENGTH_M - 4.4,
        // !isRedSide() ? 0.45 : FIELD_WIDTH_M - 0.45, !isRedSide() ?
        // Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg)),
        new ParallelRaceGroup(
            AutoBuilder.followPath(newRightStart),
            shooter.zeroHood(),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
        // AutoBuilder.followPath(midRightOutpostShoot),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            new SequentialCommandGroup(
                ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper)))
            .withTimeout(5)
        // new ParallelRaceGroup(
        //   AutoBuilder.followPath(rightOutpostShootToOutpost), 
        //   shooter.defaultCommand(),
        //   loader.loadBoth(0),
        //   hopper.stop()),
        //   intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED).withTimeout(2),
        // AutoBuilder.followPath(outpostToOutpostShoot),
        // new ParallelCommandGroup(
        //     ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
        //     new SequentialCommandGroup(
                // ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper)))
              );

    Command leftSideOneAuto = new SequentialCommandGroup(
        // drivetrain.resetThePose(new Pose2d(!isRedSide() ? 4.4 : FIELD_LENGTH_M - 4.4,
        // !isRedSide() ? 7.64 : FIELD_WIDTH_M - 7.64, !isRedSide() ?
        // Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg)),
        new ParallelRaceGroup(
            AutoBuilder.followPath(startLeft),
            intake.outZeroExtension()),
        new ParallelRaceGroup(
            AutoBuilder.followPath(intakeLeft),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
        AutoBuilder.followPath(midLeftShoot),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain))));

    Command leftDepotAuto = new SequentialCommandGroup(
        // drivetrain.resetThePose(new Pose2d(!isRedSide() ? 4.4 : FIELD_LENGTH_M - 4.4,
        // !isRedSide() ? 7.64 : FIELD_WIDTH_M - 7.64, !isRedSide() ?
        // Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg)),
        new ParallelRaceGroup(
            AutoBuilder.followPath(startLeft),
            intake.outZeroExtension()),
        new ParallelRaceGroup(
            AutoBuilder.followPath(intakeLeft),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
        AutoBuilder.followPath(MidleftToDepotShoot),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            new SequentialCommandGroup(
                ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper)))
            .withTimeout(5),
        new ParallelRaceGroup(
            AutoBuilder.followPath(DepotShootToDepot),
            shooter.defaultCommand(),
            loader.loadBoth(0),
            hopper.stop()),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED).withTimeout(2.5),
        AutoBuilder.followPath(DepotToShoot),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            new SequentialCommandGroup(
                ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper))));

    Command terry = new SequentialCommandGroup(
        AutoBuilder.followPath(PutItInReverseTerry),
        drivetrain.applyRequest(() -> brake));

    Command BoeingRight = new SequentialCommandGroup(
        AutoBuilder.followPath(AirplaneTakeDownRight));

    Command BoeingLeft = new SequentialCommandGroup(
        AutoBuilder.followPath(AirplaneTakeDownLeft));

    Command NewRightSide = new SequentialCommandGroup(
        AutoBuilder.followPath(RightControledChaos));

    // autoChooser.setDefaultOption("Example", exampleAuto);
    autoChooser.setDefaultOption("Right One Cycle", rightSideOneAuto);
    // autoChooser.addOption("Right One and Outpost Cycle",
    // rightSideOneAndOutpostAuto);
    autoChooser.addOption("Left One Cycle", leftSideOneAuto);
    autoChooser.addOption("Put It In Reverse Terry", terry);
    autoChooser.addOption("outpost cycle dif", rightSideOneAndOutpostAutoDif);
    autoChooser.addOption("De-Pot", leftDepotAuto);
    autoChooser.addOption("Boeing Door", BoeingRight);
    autoChooser.addOption("Boeing Cockpit", BoeingLeft);
    autoChooser.addOption("NewRight", NewRightSide);

    SmartDashboard.putData("Auto chooser", autoChooser);

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
