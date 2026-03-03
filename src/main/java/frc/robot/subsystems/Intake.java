package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                .withStatorCurrentLimitEnable(false)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(60)
                .withSupplyCurrentLowerTime(5.0))
        .withIdleMode(NeutralModeValue.Coast)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PIDSGAV(5, 0, 0, 12, 0, 0, 0)// 0.12413 * 24.0 * 2.8 / 11.0)
        .build();
    intakeMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(24.0 / 11.0));

    extensionMotor = KrakenBuilder.create(INTAKE_MOTOR_2_ID, "rio", "Intake", "Actuation Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(10))
        .withIdleMode(NeutralModeValue.Brake)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PIDSGAV(5, 0, 0, 0, 0, 0, 0.12413 * 46.0 / 11.0)
        .build();

    // extensionMotor.getConfigurator()
    // .apply(new FeedbackConfigs().withSensorToMechanismRatio(46.0 / (11.0)));

    // extensionMotor.setGearRatio(46.0 / (11.0 * 3.0 * Math.PI));
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

  public Command setIntakeState(DoubleSupplier rotationsDistance, DoubleSupplier rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance.getAsDouble();
          desiredIntakeSpeed = rotationsPerSecond.getAsDouble();
          extensionMotor.setControl(new PositionVoltage(rotationsDistance.getAsDouble() - 0.1).withEnableFOC(true));
          intakeMotor.setControl(new VelocityVoltage(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
        });
  }

  public Command zeroExtension() {
    return run(
        () -> {
          desiredExtensionRotations = 0;
          extensionMotor.setControl(new VoltageOut(-6).withEnableFOC(true));
          extensionMotor.setPosition(0);
        });
  }

  public Command outZeroExtension() {
    return run(
        () -> {
          desiredExtensionRotations = EXTEND_DISTANCE;
          extensionMotor.setControl(new VoltageOut(6).withEnableFOC(true));
          extensionMotor.setPosition(EXTEND_DISTANCE);
          intakeMotor.setControl(new VoltageOut(0));
        });
  }

  public Command setDefaultCommand() {
    return run(
        () -> {
          desiredIntakeSpeed = 0;
          desiredExtensionRotations = extensionMotor.getPosition().getValueAsDouble();
          extensionMotor
              .setControl(new PositionVoltage(extensionMotor.getPosition().getValueAsDouble()).withEnableFOC(true));
          intakeMotor.setControl(new VoltageOut(0).withEnableFOC(true));
        });
  }

  public Command agitateIntake() {
    return new SequentialCommandGroup(
        setIntakeState(6, 35).withTimeout(0.1),
        setIntakeState(EXTEND_DISTANCE - 0.075, 35).withTimeout(0.1)).repeatedly();
  }

  @Override
  public void periodic() {
    DogLog.log(INTAKE_KEY + "desired distance", desiredExtensionRotations);
    DogLog.log(INTAKE_KEY + "desired speed", desiredIntakeSpeed);
  }
}
