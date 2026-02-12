// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.MAX_DRIVE_SPEED;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final Telemetry logger = new Telemetry(MAX_DRIVE_SPEED);



    public RobotContainer() {
        setDefaultCommands();
        configureBindings();
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(
            drivetrain.joystickDrive(
                () -> -joystick.getLeftY() * MAX_DRIVE_SPEED, 
                () -> -joystick.getLeftX() * MAX_DRIVE_SPEED, 
                () -> joystick.getRightX() * MAX_DRIVE_SPEED)
        );

        hopper.setDefaultCommand(hopper.set(0, 0));
        intake.setDefaultCommand(intake.set(0));
        loader.setDefaultCommand(loader.set(0));
        shooter.setDefaultCommand(shooter.set(0));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }
    private void configureBindings() {
        joystick.leftTrigger().whileTrue(
          intake.set(45)  
        );

        joystick.rightTrigger().whileTrue(
            intake.extend(4)
        );

        joystick.leftBumper().whileTrue(
            hopper.set(84, 80)
        );

        joystick.a().whileTrue(
            shooter.set(50)
        );

        joystick.x().whileTrue(
            loader.setBoth(50)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No Auto");
    }
}
