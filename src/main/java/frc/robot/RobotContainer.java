// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.MAX_DRIVE_SPEED;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_LOWER_SPEED;
import static frc.robot.constants.SubsystemConstants.HopperConstants.HOPPER_UPPER_SPEED;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;

import java.nio.file.Path;
import java.util.function.Supplier;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommandsCopy;
import frc.robot.commands.ShooterCommandsCopyCopy;
// import frc.robot.commands.ShooterCommandsS;
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
  private SendableChooser<PathPlannerPath> pathThree;

  public static final CommandXboxController joystick = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MAX_DRIVE_SPEED);

  public Command autonomousSequence;

  public RobotContainer() {
    setDefaultCommands();
    configureBindings();

    setupAutos();

    PathfindingCommand.warmupCommand().schedule();
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
    // joystick.leftTrigger().whileTrue(intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED));// checkmark
    joystick.leftTrigger().whileTrue(IntakeCommands.intakeCommand(intake, drivetrain, joystick::getLeftX, joystick::getLeftY, joystick::getRightX));
    
    joystick.rightTrigger().whileTrue(intake.barf());// checkmark

    // joystick.leftBumper().whileTrue(ShooterCommandsS.aim(shooter, drivetrain,
    //         joystick::getLeftX, joystick::getLeftY));
    // joystick.leftBumper().whileTrue(ShooterCommandsS.fire(loader, intake, hopper,
    //         shooter, drivetrain));
    // joystick.leftBumper().whileTrue(ShooterCommandsS.dejam(loader, hopper,
    // intake));

    // shoot rev
    // joystick.a().whileTrue(
    //     ShooterCommands.teleHalfShooterCommand(shooter,
    //         drivetrain, joystick::getLeftX,
    //         joystick::getLeftY)); // IT WORKS!!!!
    // // shoot with intake agitation
    // joystick.a().whileTrue(
    //     ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain));
    joystick.povUp().whileTrue(
        intake.agitateIntakeMM()
    );

    // joystick.a().and(drivetrain::isAtDesiredAngle).whileTrue(
    // drivetrain.applyRequest(() -> brake)
    // );
    
    joystick.povLeft().whileTrue(
        intake.outZeroExtension());

    joystick.povRight().whileTrue(
        shooter.zeroHood());

    joystick.a().whileTrue(
        ShooterCommandsCopy.teleHalfShooterCommand(shooter, drivetrain, joystick::getLeftX, joystick::getLeftY));
    joystick.a().whileTrue(
        ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain)
    );
    joystick.a().and(joystick.leftTrigger()).whileTrue(
      ShooterCommandsCopy.tele2ndHalfShooterCommandWithIntake(loader, intake, hopper, shooter, drivetrain)
    );

    joystick.leftTrigger().and(joystick.a()).whileTrue(
        ShooterCommandsCopy.tele2ndHalfShooterCommandWithIntake(loader, intake, hopper, shooter, drivetrain)
      );

    joystick.y().whileTrue(drivetrain.applyRequest(() -> brake));// sure

    joystick.b().whileTrue(ShooterCommands.ffShuttle(shooter, drivetrain, loader, intake, hopper, joystick::getLeftX,
            joystick::getLeftY));
    // joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));// not
    // me

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // testing commands
    operator.rightStick().whileTrue(shooter.oof());
    operator.rightBumper().onTrue(ShooterCommandsCopyCopy.addShooterAdd(shooter));
    operator.leftBumper().onTrue(ShooterCommandsCopyCopy.subtractShooterAdd(shooter));

    joystick.x().whileTrue(ShooterCommandsCopyCopy.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0));
    joystick.x().whileTrue(ShooterCommands.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain));

    operator.povLeft().whileTrue(intake.outZeroExtension());
    operator.povDown().whileTrue(intake.setIntakeState(0,0));
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

    operator.b().whileTrue(ShooterCommands.ffShuttleNoDrive(shooter, drivetrain, loader, intake, hopper, joystick::getLeftX, joystick::getLeftY));

    operator.a().whileTrue(
        shooter.setShooterState(0.01, 43));

        //power conserving runner
    operator.x().whileTrue(new ParallelCommandGroup(
            hopper.agitate(HOPPER_LOWER_SPEED/2, HOPPER_UPPER_SPEED),
        loader.loadBoth(40)).repeatedly());
    operator.y().whileTrue(new ParallelCommandGroup(
            intake.agitateIntake(),
        hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED),
        loader.loadBoth(40)).repeatedly());
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
    pathThree = new SendableChooser<>();
    SmartDashboard.putData("Auto chooser", autoChooser);
    SmartDashboard.putData("First Path", pathOne);
    SmartDashboard.putData("Second Path", pathTwo);
    SmartDashboard.putData("Third Path", pathThree);

    // SmartDashboard.putNumber("ANGLE", 0.02);
    // SmartDashboard.putNumber("SPEED", 50);

    SmartDashboard.putNumber("Wait Time", 0.0);
    SmartDashboard.putNumber("Shoot One", 5.0);
    SmartDashboard.putNumber("Shoot Two", 7.0);
    SmartDashboard.putNumber("Shoot Three", 4.0);

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
    PathPlannerPath secondSwipShallow;
    PathPlannerPath secondSwipFar;
    PathPlannerPath secondSwipFarTrench;
    PathPlannerPath depot;
    PathPlannerPath ZoneStartBump;
    PathPlannerPath ZoneStartTrench;
    PathPlannerPath newDepot;
    PathPlannerPath depAutoStart;
    PathPlannerPath acrossTheBumb;
    PathPlannerPath secondSwipTrench;
    PathPlannerPath middle;
    PathPlannerPath depAutoZoneStart;
    PathPlannerPath OnlyBumpFollow;
    PathPlannerPath SecondSwipeShallowTrench;

    try {
      secondSwipFarTrench = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path_Far_Trench");
      secondSwipFar = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path_Far");
      secondSwip = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path");
      secondSwipShallow = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path_shallow");
      newStart = PathPlannerPath.fromChoreoTrajectory("New_Right_Start");
      ControledChaos = PathPlannerPath.fromChoreoTrajectory("IDk_what_to_call_this");
      depot = PathPlannerPath.fromChoreoTrajectory("Depot");
      ZoneStartBump = PathPlannerPath.fromChoreoTrajectory("Zone_Start_Bump");
      ZoneStartTrench = PathPlannerPath.fromChoreoTrajectory("Zone_Start_Trench");
      newDepot = PathPlannerPath.fromChoreoTrajectory("New_depot");
      depAutoStart = PathPlannerPath.fromChoreoTrajectory("Depauto_Start");
      acrossTheBumb = PathPlannerPath.fromChoreoTrajectory("across_The_Bumb");
      secondSwipTrench = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path_Trench");
      middle = PathPlannerPath.fromChoreoTrajectory("Middle");
      depAutoZoneStart = PathPlannerPath.fromChoreoTrajectory("Depauto_Zone_Start");
      OnlyBumpFollow = PathPlannerPath.fromChoreoTrajectory("Zone_Start_Only_Bump");
      SecondSwipeShallowTrench = PathPlannerPath.fromChoreoTrajectory("Right_2nd_Path_Trench_Shallow");
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

    autonomousSequence = new SequentialCommandGroup(
            Commands.defer(getWaitCommand(), Set.of()),
            new ParallelRaceGroup(
                    new SequentialCommandGroup(
                            intake.zeroExtension().withTimeout(0.2),
                            intake.setIntakeState(2, 0).withTimeout(.3),
                            intake.setIntakeState(0, 0).withTimeout(0.2),
                            intake.zeroExtension().withTimeout(0.1),
                            intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
                    shooter.zeroHood(),
                    drivetrain.defer(() -> AutoBuilder.followPath(getFirstPath()))),
            new ParallelRaceGroup(
                    ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
                    ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain),
                    Commands.defer(() -> new WaitCommand(SmartDashboard.getNumber("Shoot One", 5.0)), Set.of())),
            new ParallelRaceGroup(
                    drivetrain.defer(() -> AutoBuilder.followPath(getSecondPath())),
                    shooter.defaultCommand(),
                    loader.loadBoth(0),
                    hopper.stop(),
                    intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
            new ParallelRaceGroup(
                    ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
                    ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain),
                    Commands.defer(() -> new WaitCommand(SmartDashboard.getNumber("Shoot Two", 7.0)), Set.of())),
            new ParallelRaceGroup(
                    drivetrain.defer(() -> AutoBuilder.followPath(getThirdPath())),
                    shooter.defaultCommand(),
                    loader.loadBoth(0),
                    hopper.stop(),
                    intake.setIntakeState(EXTEND_DISTANCE, INTAKE_SPEED)),
            new ParallelRaceGroup(
                    ShooterCommands.teleHalfShooterCommand(shooter, drivetrain, () -> 0, () -> 0),
                    ShooterCommandsCopy.tele2ndHalfShooterCommand(loader, intake, hopper, shooter, drivetrain),
                    Commands.defer(() -> new WaitCommand(SmartDashboard.getNumber("Shoot Three", 4.0)), Set.of())));

    pathOne.setDefaultOption("Start", newStart);
    pathOne.addOption("controlled chaos", ControledChaos);
    pathOne.addOption("DepAuto Start", depAutoStart);
    pathOne.addOption("Zone Start Bump", ZoneStartBump);
    pathOne.addOption("Zone Start Trench", ZoneStartTrench);
    pathOne.addOption(("Middle Start"), middle);
    pathOne.addOption("Depauto Zone Start", depAutoZoneStart);
    pathOne.addOption("Only Bump Follow", OnlyBumpFollow);

    pathTwo.setDefaultOption("second path close", secondSwip);
    pathTwo.addOption("second path far", secondSwipFar);
    pathTwo.addOption("second path far trench", secondSwipFarTrench);
    pathTwo.addOption("second Swip Shallow", secondSwipShallow);
    pathTwo.addOption("normal depot",depot);
    pathTwo.addOption("Shooting on the fly Depot", newDepot);
    pathTwo.addOption("Across The Bumb", acrossTheBumb);
    pathTwo.addOption("Second Swip Trench", secondSwipTrench);
    pathTwo.addOption("Right_2nd_Path_Trench_Shallow", SecondSwipeShallowTrench);


    pathThree.setDefaultOption("second path close", secondSwip);
    pathThree.addOption("second path far", secondSwipFar);
    pathThree.addOption("second path far trench", secondSwipFarTrench);
    pathThree.addOption("second Swip Shallow", secondSwipShallow);
    pathThree.addOption("Depot", newDepot);
    pathThree.addOption("Across The Bumb", acrossTheBumb);
    pathThree.addOption("Second Swip Trench", secondSwipTrench);
    pathThree.addOption("normal depot",depot);

    // autoChooser.setDefaultOption("Example", exampleAuto);
    // autoChooser.addOption("Right One and Outpost Cycle",
    // rightSideOneAndOutpostAuto);
    autoChooser.setDefaultOption("Mix & Match Left", new InstantCommand(() -> {
    }).withName("Mix Left"));
    autoChooser.addOption("Mix & Match Right", new InstantCommand(() -> {
    }).withName("Mix Right"));
    // autoChooser.addOption("Test for mix&match", PathOne);

    SmartDashboard.putData("Auto chooser", autoChooser);
    SmartDashboard.putData("First Path", pathOne);
    SmartDashboard.putData("Second Path", pathTwo);

}

public PathPlannerPath getFirstPath() {
    if (autoChooser.getSelected().getName().equals("Mix Left")) {
        return pathOne.getSelected().mirrorPath();
    } else {
        return pathOne.getSelected();
    }
}

public PathPlannerPath getSecondPath() {
    if (autoChooser.getSelected().getName().equals("Mix Left")) {
        return pathTwo.getSelected().mirrorPath();
    } else {
        return pathTwo.getSelected();
    }
}

public PathPlannerPath getThirdPath() {
    if (autoChooser.getSelected().getName().equals("Mix Left")) {
        return pathThree.getSelected().mirrorPath();
    } else {
        return pathThree.getSelected();
    }
}

public Supplier<Command> getWaitCommand() {
    return () -> new WaitCommand(SmartDashboard.getNumber("Wait Time", 0.0));
}

public Supplier<Command> getShootOne() {
    return () -> new WaitCommand(SmartDashboard.getNumber("Wait Time", 0.0));
}

public Supplier<Command> getShootTwo() {
    return () -> new WaitCommand(SmartDashboard.getNumber("Wait Time", 0.0));
}

public Command getAutonomousCommand() {
    if (autoChooser.getSelected().getName().equals("Mix Left")) {
        return autonomousSequence;
    } else if (autoChooser.getSelected().getName().equals("Mix Right")) {
        return autonomousSequence;
    } else {
        return autoChooser.getSelected();
    }
  }
}
