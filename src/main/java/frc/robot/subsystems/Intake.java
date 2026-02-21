package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.*;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  Kraken intakeMotor;
  Kraken extensionMotor;

  double desiredIntakeSpeed;
  double desiredExtensionRotations;

  public Intake() {
    intakeMotor = KrakenBuilder.create(INTAKE_MOTOR_ID, "rio", "Intake", "Intake Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PIDSGAV(5, 0, 0, 0, 0, 0, 0.12413 * 24.0 / 11.0)
        .build();
    intakeMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(24.0 / 11.0));

    extensionMotor = KrakenBuilder.create(INTAKE_MOTOR_2_ID, "rio", "Intake", "Actuation Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Brake)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PIDSGAV(5, 0, 0, 0, 0, 0, 0.12413 * 46.0 / 11.0)
        .build();
    intakeMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(46.0 / 11.0));
  }

  /**
   * @param speed in rotations per second
   * @return
   */
  public Command intake(double speed) {
    return run(() -> {
      this.desiredIntakeSpeed = speed;
      intakeMotor.setControl(
          new VelocityVoltage(speed).withEnableFOC(true));
    });
  }

  /**
   * @param speed in rotations per second
   * @return
   */
  public Command intake(DoubleSupplier speed) {
    return run(
        () -> {
          this.desiredIntakeSpeed = speed.getAsDouble();
          intakeMotor.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true));
        });
  }

  /**
   * @param distance in inches of the actuation distance
   * @return
   */
  public Command extend(double rotations) {
    this.desiredExtensionRotations = rotations;
    return run(() -> extensionMotor.setControl(new PositionVoltage(rotations).withEnableFOC(true)));
  }

  /**
   * @param distance in inches of the actuation distance as a supplier
   * @return
   */
  public Command extend(DoubleSupplier rotations) {
    this.desiredExtensionRotations = rotations.getAsDouble();
    return run(
        () -> extensionMotor.setControl(new PositionVoltage(this.desiredExtensionRotations).withEnableFOC(true)));
  }

  public Command setIntakeState(double rotationsDistance, double rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance;
          desiredIntakeSpeed = rotationsPerSecond;
          extensionMotor.setControl(new PositionVoltage(rotationsDistance).withEnableFOC(true));
          intakeMotor.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
        });
  }

  public Command defaultCommand() {
    return run(
        () -> {
          setIntakeState(desiredExtensionRotations, 0);
        });
  }

  public Command zeroHood() {
    return run(
        () -> {
          extensionMotor.setControl(new VoltageOut(1).withEnableFOC(true));
          extensionMotor.setPosition(extensionMotor.getPosition().getValue());
        });
  }

  @Override
  public void periodic() {
    DogLog.log(INTAKE_KEY + "distance", desiredExtensionRotations);
    DogLog.log(INTAKE_KEY + "speed", desiredIntakeSpeed);
  }
}
