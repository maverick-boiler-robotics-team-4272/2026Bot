// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.MAX_DRIVE_SPEED;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_LOWER_SPEED;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_UPPER_SPEED;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private SendableChooser<PathPlannerPath> pathOne;
  private SendableChooser<PathPlannerPath> pathTwo;

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
    joystick.rightTrigger().whileTrue(intake.barf());// checkmark
    joystick.leftBumper().whileTrue(intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED));// checkmark
    joystick.rightTrigger().whileTrue(intake.barf());

    // joystick.leftBumper().whileTrue(drivetrain.doTrenches());

    // shoot rev
    joystick.a().whileTrue(
        ShooterCommands.teleHalfShooterCommand(shooter,
            drivetrain, joystick::getLeftX,
            joystick::getLeftY)); // IT WORKS!!!!
    // shoot with intake agitation
    joystick.a().whileTrue(
        ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain));

    // joystick.a().and(drivetrain::isAtDesiredAngle).whileTrue(
    // drivetrain.applyRequest(() -> brake)
    // );
    // shoot and intake fuel
    joystick.a().and(joystick.leftTrigger()).whileTrue(
        ShooterCommands.tele2ndHalfShooterCommandWithIntake(loader, intake, hopper, shooter, drivetrain));

    joystick.povLeft().whileTrue(
        intake.outZeroExtension());

    joystick.povRight().whileTrue(
        shooter.zeroHood());
    joystick.rightBumper().whileTrue(
        ShooterCommandsCopy.teleHalfShooterCommand(shooter, drivetrain, joystick::getLeftX, joystick::getLeftY));
    joystick.rightBumper().whileTrue(
        ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain));
    // joystick.x().whileTrue(ShooterCommandsCopyCopy.teleHalfShooterCommand(shooter,
    // drivetrain, joystick::getLeftX, joystick::getLeftY));
    // joystick.x().whileTrue(ShooterCommandsCopyCopy.tele2ndHalfShooterCommand(loader,
    // intake, hopper, shooter, drivetrain));
    // joystick.y().whileTrue(ShooterCommandsCopy.tele2ndHalfShooterCommand(loader,
    // intake, hopper, shooter, drivetrain));

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
    operator.rightStick().whileTrue(shooter.oof());
    // operator.rightBumper().onTrue(ShooterCommandsCopyCopy.addShooterAdd(shooter));
    // operator.leftBumper().onTrue(ShooterCommandsCopyCopy.subtractShooterAdd(shooter));

    operator.povLeft().whileTrue(intake.outZeroExtension());
    operator.povDown().whileTrue(intake.zeroExtension());
    operator.povRight().whileTrue(shooter.zeroHood());
    operator.povUp().whileTrue(hopper.agitate(-HOPPER_LOWER_SPEED, -HOPPER_UPPER_SPEED));
    operator.leftStick().whileTrue(hopper.agitate(-5, 0));

    operator.rightTrigger().whileTrue(
        intake.agitateIntake());
    operator.rightBumper().whileTrue(
        intake.stupidateIntake());

    // operator.y().whileTrue(
    // shooter.setShooterState(0.015, 50));
    // operator.rightTrigger().whileTrue(new PathPlannerAuto("Test", false));

    drivetrain.registerTelemetry(logger::telemeterize);

    // operator.b().whileTrue(
    // shooter.setShooterState(0.07, 60));

    // operator.x().whileTrue(
    // hopper.agitate(-50, -30));

    operator.b().whileTrue(shooter.setShooterState(() -> SmartDashboard.getNumber("ANGLE", 0.02),
        () -> SmartDashboard.getNumber("SPEED", 50)));

    operator.a().whileTrue(
        shooter.setShooterState(0.01, 43));

    operator.x().whileTrue(new ParallelCommandGroup(
        intake.agitateIntake(),
        hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
        loader.loadBoth(70)).repeatedly());
    operator.y().whileTrue(new ParallelCommandGroup(
        intake.agitateIntake(),
        hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
        loader.loadBoth(70)).repeatedly());
  }

  // public void registerNamedCommands() {
  // // NamedCommands.registerCommand("EXAMPLE", command);
  // NamedCommands.registerCommand(
  // "Intake",
  // intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED));
  // }

  public void setupAutos() {
    autoChooser = new SendableChooser<>();
    pathOne = new SendableChooser<>();
    pathTwo = new SendableChooser<>();
    SmartDashboard.putData("Auto chooser", autoChooser);
    SmartDashboard.putData("First Path", pathOne);
    SmartDashboard.putData("Second Path", pathTwo);

    SmartDashboard.putNumber("ANGLE", 0.02);
    SmartDashboard.putNumber("SPEED", 50);
    SmartDashboard.putNumber("Wait Time", 0.0);

    // PathPlannerPath ExamplePath;
    // try {
    // ExamplePath = PathPlannerPath.fromChoreoTrajectory("Exact Name of Path");
    // } catch (Exception e) {
    // throw new RuntimeException("Failed to load Choreo trajectory: " +
    // e.getMessage());
    // }

    PathPlannerPath ControledChaos;
    PathPlannerPath newStart;
    PathPlannerPath secondSwip;
    PathPlannerPath secondSwipFar;
    PathPlannerPath secondSwipFarTrench;
    PathPlannerPath fistMyBumpRight;
    PathPlannerPath fistMyBumpLeft;
    PathPlannerPath fistBumpRight;
    PathPlannerPath fistBumpLeft;

    try {
      fistBumpLeft = PathPlannerPath.fromChoreoTrajectory("Fist_Bump").mirrorPath();
      fistBumpRight = PathPlannerPath.fromChoreoTrajectory("Fist_Bump");
      fistMyBumpLeft = PathPlannerPath.fromChoreoTrajectory("Fist_My_Bump").mirrorPath();
      fistMyBumpRight = PathPlannerPath.fromChoreoTrajectory("Fist_My_Bump");
      secondSwipFarTrench = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path_Far_Trench");
      secondSwipFar = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path_Far");
      secondSwip = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path");
      newStart = PathPlannerPath.fromChoreoTrajectory("New_Right_Start");
      ControledChaos = PathPlannerPath.fromChoreoTrajectory("IDk_what_to_call_this");
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

    // Command doubleSwipRight = new SequentialCommandGroup(
    // new ParallelRaceGroup(
    // AutoBuilder.followPath(newRightStart),
    // shooter.zeroHood(),
    // intake.setIntakeStateOUT(EXTEND_DISTANCE, INTAKE_SPEED)),
    // // AutoBuilder.followPath(midRightOutpostShoot),
    // new ParallelCommandGroup(
    // ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () ->
    // 0),
    // ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper,
    // shooter, drivetrain))
    // .withTimeout(4),
    // new ParallelRaceGroup(
    // AutoBuilder.followPath(secondSwipRight),
    // shooter.defaultCommand(),
    // loader.loadBoth(0),
    // hopper.stop(),
    // intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
    // new ParallelCommandGroup(
    // ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () ->
    // 0),
    // ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper,
    // shooter, drivetrain)));

    Command HAILMARY_Right = new SequentialCommandGroup(
        new ParallelRaceGroup(
            intake.setIntakeStateOUT(EXTEND_DISTANCE, INTAKE_SPEED),
            new SequentialCommandGroup(
                AutoBuilder.followPath(fistMyBumpRight),
                drivetrain.applyRequest(() -> brake).withTimeout(2),
                AutoBuilder.followPath(fistBumpRight))),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain)).withTimeout(4),
        new ParallelRaceGroup(
            AutoBuilder.followPath(secondSwip),
            shooter.defaultCommand(),
            loader.loadBoth(0),
            hopper.stop(),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain)));

    Command HAILMARY_Left = new SequentialCommandGroup(
        new ParallelRaceGroup(
            intake.setIntakeStateOUT(EXTEND_DISTANCE, INTAKE_SPEED),
            new SequentialCommandGroup(
                AutoBuilder.followPath(fistMyBumpLeft),
                drivetrain.applyRequest(() -> brake).withTimeout(2),
                AutoBuilder.followPath(fistBumpLeft))),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain)).withTimeout(4),
        new ParallelRaceGroup(
            AutoBuilder.followPath(secondSwip.mirrorPath()),
            shooter.defaultCommand(),
            loader.loadBoth(0),
            hopper.stop(),
            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
        new ParallelCommandGroup(
            ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
            ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain)));

    pathOne.setDefaultOption("Start", newStart);
    pathOne.addOption("controlled chaos", ControledChaos);

    pathTwo.setDefaultOption("second path close", secondSwip);
    pathTwo.addOption("second path far", secondSwipFar);
    pathTwo.addOption("second path far trench", secondSwipFarTrench);

    // autoChooser.setDefaultOption("Example", exampleAuto);
    // autoChooser.addOption("Right One and Outpost Cycle",
    // rightSideOneAndOutpostAuto);

    autoChooser.addOption("HailMary Left", HAILMARY_Left);
    autoChooser.addOption("HailMary Right", HAILMARY_Right);
    autoChooser.addOption("Mix & Match Left", new InstantCommand(() -> {
    }).withName("Mix Left"));
    autoChooser.addOption("Mix & Match Right", new InstantCommand(() -> {
    }).withName("Mix Right"));
    // autoChooser.addOption("Test for mix&match", PathOne);

    SmartDashboard.putData("Auto chooser", autoChooser);
    SmartDashboard.putData("First Path", pathOne);
    SmartDashboard.putData("Second Path", pathTwo);

  }

  public Command getAutonomousCommand() {
    if (autoChooser.getSelected().getName().equals("Mix Left")) {
      return new SequentialCommandGroup(
          new WaitCommand(SmartDashboard.getNumber("Wait Time", 0.0)),
          new ParallelRaceGroup(
              new SequentialCommandGroup(
                  intake.zeroExtension().withTimeout(0.2),
                  intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
                  shooter.zeroHood(),
              AutoBuilder.followPath(pathOne.getSelected().mirrorPath())),
          new ParallelCommandGroup(
              ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
              ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain))
              .withTimeout(5),
          new ParallelRaceGroup(
              AutoBuilder.followPath(pathTwo.getSelected().mirrorPath()),
              shooter.defaultCommand(),
              loader.loadBoth(0),
              hopper.stop(),
              intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
          new ParallelCommandGroup(
              ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
              ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain))
              .withTimeout(5));
    } else if (autoChooser.getSelected().getName().equals("Mix Right")) {
      return new SequentialCommandGroup(
          new WaitCommand(SmartDashboard.getNumber("Wait Time", 0.0)),
          new ParallelRaceGroup(
              new SequentialCommandGroup(
                  intake.zeroExtension().withTimeout(0.2),
                  intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
                  shooter.zeroHood(),
              AutoBuilder.followPath(pathOne.getSelected())),
          new ParallelCommandGroup(
              ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
              ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain))
              .withTimeout(5),
              new ParallelRaceGroup(
                AutoBuilder.followPath(pathTwo.getSelected()),
                shooter.defaultCommand(),
                loader.loadBoth(0),
                hopper.stop(),
                intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
          new ParallelCommandGroup(
              ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
              ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain))
              .withTimeout(5));
    }
    return autoChooser.getSelected();
  }
}
