package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  Kraken intakeMotor;
  Kraken extensionMotor;

  double desiredIntakeSpeed;
  double desiredExtensionRotations;

  CurrentLimitsConfigs regularLimits = new CurrentLimitsConfigs();
  CurrentLimitsConfigs safeLimits = new CurrentLimitsConfigs();

  double prevDesDistance;
  boolean disableSafety;

  Trigger currentLimitTrigger;

  public Intake() {
    regularLimits.StatorCurrentLimitEnable = false;
    regularLimits.SupplyCurrentLimit = 30;
    regularLimits.SupplyCurrentLimitEnable = true;
    regularLimits.SupplyCurrentLowerLimit = 20;

    safeLimits.StatorCurrentLimit = 5;
    safeLimits.StatorCurrentLimitEnable = true;
    safeLimits.SupplyCurrentLimit = 5;
    safeLimits.SupplyCurrentLimitEnable = true;
    safeLimits.SupplyCurrentLowerLimit = 0;

    disableSafety = false;

    intakeMotor = KrakenBuilder.create(INTAKE_MOTOR_ID, "rio", "Intake", "Intake Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(false)
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(60)
                .withSupplyCurrentLowerTime(3.0))
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
        .withIdleMode(NeutralModeValue.Coast)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PIDSGAV(5, 0, 0, 0, 0, 0, 0.12413 * 46.0 / 11.0)
        .build();
    extensionMotor.getConfigurator().apply(new Slot1Configs().withKP(0.01));
    prevDesDistance = extensionMotor.getPosition().getValueAsDouble();

    extensionMotor.setPosition(0);
    // extensionMotor.getConfigurator()
    // .apply(new FeedbackConfigs().withSensorToMechanismRatio(46.0 / (11.0)));

    // extensionMotor.setGearRatio(46.0 / (11.0 * 3.0 * Math.PI));

    extensionMotor.getConfigurator()
        .apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(4)
            .withMotionMagicAcceleration(9999));

    currentLimitTrigger = new Trigger(this::isSafe);

    // currentLimitTrigger.onTrue(new InstantCommand(() -> {
    //   extensionMotor.getConfigurator().apply(safeLimits);
    // }));

    // currentLimitTrigger.onFalse(new InstantCommand(() -> {
    //   extensionMotor.getConfigurator().apply(regularLimits);
    // }));
  }

  public Command setIntakeState(double rotationsDistance, double rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance;
          desiredIntakeSpeed = rotationsPerSecond;
          if (!isSafe()) {
            extensionMotor.setControl(new PositionVoltage(rotationsDistance).withEnableFOC(true).withSlot(0));
          } else {
            extensionMotor.setControl(new PositionVoltage(rotationsDistance).withEnableFOC(true).withSlot(1));
          }
          intakeMotor.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
        });
  }

  public Command setIntakeStateOUT(double rotationsDistance, double rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance;
          desiredIntakeSpeed = rotationsPerSecond;
          extensionMotor.setControl(new VoltageOut(12).withEnableFOC(true));
          intakeMotor.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
        }).finallyDo(() -> extensionMotor.setPosition(EXTEND_DISTANCE));
  }

  public Command setIntakeState(DoubleSupplier rotationsDistance, DoubleSupplier rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance.getAsDouble();
          desiredIntakeSpeed = rotationsPerSecond.getAsDouble();
          if (!isSafe()) {
            extensionMotor.setControl(new PositionVoltage(rotationsDistance.getAsDouble() - 0.1).withSlot(0).withEnableFOC(true));
          } else {
            extensionMotor.setControl(new PositionVoltage(rotationsDistance.getAsDouble() - 0.1).withSlot(1).withEnableFOC(true));
          }
          intakeMotor.setControl(new VelocityVoltage(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
        });
  }

  public Command setIntakeStateMotionMagic(double rotationsDistance, double rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance;
          desiredIntakeSpeed = rotationsPerSecond;
          if (!isSafe()) {
            extensionMotor.setControl(
                new MotionMagicVoltage(rotationsDistance - 0.1).withSlot(0).withEnableFOC(true));
          } else {
            extensionMotor.setControl(
                new MotionMagicVoltage(rotationsDistance - 0.1).withSlot(1).withEnableFOC(true));
          }
          intakeMotor.setControl(new VoltageOut(0));
        });
  }

  public Command zeroExtension() {
    return startEnd(
        () -> {
          disableSafety = false;
          desiredExtensionRotations = 0;
          extensionMotor.setControl(new VoltageOut(-6).withEnableFOC(false));
        }, () -> {
          disableSafety  = true;
          extensionMotor.setPosition(0);
        });
  }

  public Command outZeroExtension() {
    return startEnd(
        () -> {
          disableSafety = true;
          desiredExtensionRotations = EXTEND_DISTANCE;
          extensionMotor.setControl(new VoltageOut(12).withEnableFOC(false));
          intakeMotor.setControl(new VoltageOut(0));
        }, () -> {
          extensionMotor.setPosition(EXTEND_DISTANCE);
          disableSafety = false;
        });
  }

  public Command setDefaultCommand() {
    return run(
        () -> {
          desiredIntakeSpeed = 0;
          desiredExtensionRotations = extensionMotor.getPosition(false).getValueAsDouble();
          extensionMotor
              .setControl(new PositionVoltage(extensionMotor.getPosition().getValueAsDouble()).withSlot(1).withEnableFOC(true));
          intakeMotor.setControl(new VoltageOut(0).withEnableFOC(true));
        });
  }

  public Command agitateIntake() {
  return new SequentialCommandGroup(
  setIntakeState(6, 45).withTimeout(0.2),
  setIntakeState(EXTEND_DISTANCE - 0.1, 45).withTimeout(0.2)).repeatedly()
  .beforeStarting(() -> {
  disableSafety = true;
  }).finallyDo(() -> disableSafety = false);
  }

  public Command agitateIntakeMM() {
    return setIntakeStateMotionMagic(0.0, 0.0)
        .beforeStarting(() -> {
          disableSafety = true;
        }).finallyDo(() -> disableSafety = false);
  }

  public Command barf() {
    return setIntakeState(EXTEND_DISTANCE, -INTAKE_SPEED);
  }

  public Command disableSafteyWhileCalled() {
    return startEnd(
        () -> {
          disableSafety = true;
          extensionMotor.getConfigurator().apply(regularLimits);
        }, () -> disableSafety = false);
  }

  public void safetyLogic() {
    if (disableSafety) {
      return;
    }
    if (prevDesDistance == desiredExtensionRotations) {
      extensionMotor.getConfigurator().apply(safeLimits);
    } else {
      extensionMotor.getConfigurator().apply(regularLimits);
    }
  }

  @Override
  public void periodic() {
    DogLog.log(INTAKE_KEY + "desired distance", desiredExtensionRotations);
    DogLog.log(INTAKE_KEY + "desired speed", desiredIntakeSpeed);
    DogLog.log(INTAKE_KEY + "isSafe", isSafe());
  }

  public boolean isSafe() {
    if (disableSafety) {
      return false;
    }
    if (prevDesDistance >= extensionMotor.getPosition(false).getValueAsDouble() - 0.1
        && prevDesDistance <= extensionMotor.getPosition(false).getValueAsDouble() + 0.1) {
      return true;
    } else {
      return false;
    }
  }
}
