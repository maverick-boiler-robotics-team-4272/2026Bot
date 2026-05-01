package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  Kraken intakeMotor;
  Kraken extensionMotor;
  Kraken extensionMotor2;

  double desiredIntakeSpeed;
  double desiredExtensionRotations;

  CurrentLimitsConfigs regularLimits = new CurrentLimitsConfigs();
  CurrentLimitsConfigs safeLimits = new CurrentLimitsConfigs();

  double prevDesDistance;
  boolean disableSafety;

  Timer zeroTimer;

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
        .withSlot0PIDSGAV(5, 0, 0, 12, 0, 0, 0)
        .build();
    intakeMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(24.0 / 11.0));

    extensionMotor = KrakenBuilder.create(EXTENSION_MOTOR_ID, "rio", "Intake", "Actuation Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(70)
                .withStatorCurrentLimitEnable(true)
            )
        .withIdleMode(NeutralModeValue.Coast)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PIDSGAV(5, 0, 0, 0, 0, 0, 0.12413 * 46.0 / 11.0)
        .build();
    extensionMotor.getConfigurator().apply(new Slot1Configs().withKP(0.01));
    extensionMotor.setPosition(0);

    extensionMotor2 = KrakenBuilder.create(EXTENSION_MOTOR_I2_D, "rio", "Intake", "Actuation Motor 2")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(70)
                .withStatorCurrentLimitEnable(true)
        )
        .withIdleMode(NeutralModeValue.Coast)
        .withInversion(InvertedValue.Clockwise_Positive)
        .withSlot0PIDSGAV(5, 0, 0, 0, 0, 0, 0.12413 * 46.0 / 11.0)
        .build();
    extensionMotor2.getConfigurator().apply(new Slot1Configs().withKP(0.01));
    extensionMotor2.setPosition(0);

    prevDesDistance = 0;// extensionMotor2.getPosition().getValueAsDouble(); // uhh...

    extensionMotor.getConfigurator()
        .apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(3)
            .withMotionMagicAcceleration(9999)
            .withMotionMagicExpo_kV(3)
            .withMotionMagicExpo_kA(0.1));

    extensionMotor2.getConfigurator()
        .apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(3)
            .withMotionMagicAcceleration(9999)
            .withMotionMagicExpo_kV(3)
            .withMotionMagicExpo_kA(0.1));
    zeroTimer = new Timer();
  }

  public Command setIntakeState(double rotationsDistance, double rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance;
          desiredIntakeSpeed = rotationsPerSecond;
          if (!isSafe()) {
            extensionMotor.setControl(new PositionVoltage(rotationsDistance).withEnableFOC(true).withSlot(0));
            extensionMotor2.setControl(new PositionVoltage(rotationsDistance).withEnableFOC(true).withSlot(0));
          } else {
            extensionMotor.setControl(new PositionVoltage(rotationsDistance).withEnableFOC(true).withSlot(1));
            extensionMotor2.setControl(new PositionVoltage(rotationsDistance).withEnableFOC(true).withSlot(1));
          }
          intakeMotor.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
        });
  }

  public Command setIntakeState(DoubleSupplier rotationsDistance, DoubleSupplier rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance.getAsDouble();
          desiredIntakeSpeed = rotationsPerSecond.getAsDouble();
          if (!isSafe()) {
            extensionMotor.setControl(new PositionVoltage(rotationsDistance.getAsDouble() - 0.1).withSlot(0).withEnableFOC(true));
            extensionMotor2
                .setControl(new PositionVoltage(rotationsDistance.getAsDouble() - 0.1).withSlot(0).withEnableFOC(true));
          } else {
            extensionMotor.setControl(new PositionVoltage(rotationsDistance.getAsDouble() - 0.1).withSlot(1).withEnableFOC(true));
            extensionMotor2
                .setControl(new PositionVoltage(rotationsDistance.getAsDouble() - 0.1).withSlot(1).withEnableFOC(true));
          }
          intakeMotor.setControl(new VelocityVoltage(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
        });
  }

  public Command driverIntake() {
    return run(
        () -> {
          zeroTimer.start();
          desiredExtensionRotations = EXTEND_DISTANCE;
          desiredIntakeSpeed = INTAKE_SPEED;

          intakeMotor.setControl(new VelocityVoltage(INTAKE_SPEED));

          if (zeroTimer.get() < 0.1) {
            extensionMotor.setControl(new VoltageOut(3));
            extensionMotor2.setControl(new VoltageOut(3));
            extensionMotor.setPosition(EXTEND_DISTANCE);
            extensionMotor2.setPosition(EXTEND_DISTANCE);
          } else {
            extensionMotor.setPosition(EXTEND_DISTANCE);
            extensionMotor2.setPosition(EXTEND_DISTANCE);
            extensionMotor.setControl(new PositionDutyCycle(EXTEND_DISTANCE));
            extensionMotor2.setControl(new PositionDutyCycle(EXTEND_DISTANCE));
          }
        }).finallyDo(() -> {
          zeroTimer.stop();
          zeroTimer.reset();
        });
  }

  public Command setExtendState(double distance) {
    return run(
      () -> {
        extensionMotor.setControl(new PositionVoltage(distance));
        extensionMotor2.setControl(new PositionVoltage(distance));
        intakeMotor.setControl(new VoltageOut(0));
      }
    );
  }

  public Command noIntakAgitate() {
    return new SequentialCommandGroup(
      setExtendState(7.5).withTimeout(0.2),
      setExtendState(EXTEND_DISTANCE - 0.1).withTimeout(0.2)).repeatedly()
      .beforeStarting(() -> {
        disableSafety = true;
      }).finallyDo(() -> disableSafety = false);
  }

  public Command zeroExtension() {
    return startEnd(
        () -> {
          disableSafety = false;
          desiredExtensionRotations = 0;
          extensionMotor.setControl(new VoltageOut(-0).withEnableFOC(false));
          extensionMotor2.setControl(new VoltageOut(-0).withEnableFOC(false));
        }, () -> {
          disableSafety  = true;
          extensionMotor.setPosition(0);
          extensionMotor2.setPosition(0);
        });
  }

  public Command outZeroExtension() {
    return startEnd(
        () -> {
          disableSafety = true;
          desiredExtensionRotations = EXTEND_DISTANCE;
          extensionMotor.setControl(new VoltageOut(1.25).withEnableFOC(false));
          extensionMotor2.setControl(new VoltageOut(1.25).withEnableFOC(false));
          intakeMotor.setControl(new VoltageOut(0));
        }, () -> {
          extensionMotor.setPosition(EXTEND_DISTANCE);
          extensionMotor2.setPosition(EXTEND_DISTANCE);
          disableSafety = false;
        }).withName("Zero");
  }

  public Command setDefaultCommand() {
    return run(
        () -> {
          desiredIntakeSpeed = 0;
          desiredExtensionRotations = extensionMotor.getPosition(false).getValueAsDouble();
          extensionMotor.setControl(
              new PositionVoltage(extensionMotor.getPosition().getValueAsDouble()).withSlot(1).withEnableFOC(true));
          extensionMotor2.setControl(
              new PositionVoltage(extensionMotor.getPosition().getValueAsDouble()).withSlot(1).withEnableFOC(true));
          intakeMotor.setControl(new VoltageOut(0).withEnableFOC(true));
        });
  }

  public Command setIntakeStateMotionMagic(double rotationsDistance, double rotationsPerSecond) {
    return run(
        () -> {
          desiredExtensionRotations = rotationsDistance;
          desiredIntakeSpeed = rotationsPerSecond;
          if (!isSafe()) {
            extensionMotor.setControl(
                new MotionMagicExpoVoltage(rotationsDistance).withSlot(0).withEnableFOC(true));
            extensionMotor2.setControl(
                new MotionMagicExpoVoltage(rotationsDistance).withSlot(0).withEnableFOC(true));
          } else {
            extensionMotor.setControl(
                new MotionMagicExpoVoltage(rotationsDistance).withSlot(1).withEnableFOC(true));
            extensionMotor2.setControl(
                new MotionMagicExpoVoltage(rotationsDistance).withSlot(1).withEnableFOC(true));
          }
          intakeMotor.setControl(new VelocityVoltage(rotationsPerSecond));
        });
  }

  public Command agitateIntake() {
    return new SequentialCommandGroup(
        setIntakeState(7.5, 45).withTimeout(0.2),
        setIntakeState(EXTEND_DISTANCE - 0.1, 45).withTimeout(0.2)).repeatedly()
        .beforeStarting(() -> {
          disableSafety = true;
        }).finallyDo(() -> disableSafety = false);
  }

  public Command agitateIntakeSlow() {
    return new SequentialCommandGroup(
        setIntakeState(7.5, 45).withTimeout(0.6),
        setIntakeState(EXTEND_DISTANCE - 0.1, 45).withTimeout(0.6)).repeatedly()
        .beforeStarting(() -> {
          disableSafety = true;
        }).finallyDo(() -> disableSafety = false);
  }

  public Command agitateIntakeMM() {
    return new SequentialCommandGroup(
        setIntakeStateMotionMagic(4, 45)).repeatedly()
        .beforeStarting(() -> {
          disableSafety = true;
        }).finallyDo(() -> disableSafety = false);
  }

  public Command stupidateIntake() {
    return new SequentialCommandGroup(
        Commands.run(() -> {
          extensionMotor.setControl(new PositionVoltage(7));
          extensionMotor2.setControl(new PositionVoltage(EXTEND_DISTANCE - 0.1));
        }).withTimeout(0.2),
        Commands.run(() -> {
          extensionMotor.setControl(new PositionVoltage(EXTEND_DISTANCE - 0.1));
          extensionMotor2.setControl(new PositionVoltage(7));
        }).withTimeout(0.2)).repeatedly().beforeStarting(
            () -> disableSafety = true)
        .finallyDo(() -> disableSafety = false);
  }

  public Command stupidateIntakeReallyStupid() {
    return new SequentialCommandGroup(
        Commands.run(() -> {
          extensionMotor.setControl(new PositionVoltage(7));
          extensionMotor2.setControl(new PositionVoltage(EXTEND_DISTANCE - 1.5));
        }).withTimeout(0.2),
        Commands.run(() -> {
          extensionMotor.setControl(new PositionVoltage(EXTEND_DISTANCE - 1.5));
          extensionMotor2.setControl(new PositionVoltage(7));
        }).withTimeout(0.2),
        Commands.run(() -> {
          extensionMotor.setControl(new PositionVoltage(EXTEND_DISTANCE - 0.1));
          extensionMotor2.setControl(new PositionVoltage(EXTEND_DISTANCE - 1.5));
        }).withTimeout(0.2),
        Commands.run(() -> {
          extensionMotor.setControl(new PositionVoltage(EXTEND_DISTANCE - 1.5));
          extensionMotor2.setControl(new PositionVoltage(EXTEND_DISTANCE - 0.1));
        }).withTimeout(0.2)).repeatedly().beforeStarting(
            () -> disableSafety = true)
        .finallyDo(() -> disableSafety = false);
  }

  public Command barf() {
    return setIntakeState(EXTEND_DISTANCE, -INTAKE_SPEED);
  }

  public Command disableSafteyWhileCalled() {
    return startEnd(
        () -> {
          disableSafety = true;
          extensionMotor.getConfigurator().apply(regularLimits);
          extensionMotor2.getConfigurator().apply(regularLimits);
        }, () -> disableSafety = false);
  }

  public void safetyLogic() {
    if (disableSafety) {
      return;
    }
    if (prevDesDistance == desiredExtensionRotations) {
      extensionMotor.getConfigurator().apply(safeLimits);
      extensionMotor2.getConfigurator().apply(safeLimits);
    } else {
      extensionMotor.getConfigurator().apply(regularLimits);
      extensionMotor2.getConfigurator().apply(regularLimits);
    }
  }

  @Override
  public void periodic() {
    DogLog.log(INTAKE_KEY + "desired distance", desiredExtensionRotations);
    DogLog.log(INTAKE_KEY + "desired speed", desiredIntakeSpeed);
    DogLog.log(INTAKE_KEY + "isSafe", isSafe());

    if (getCurrentCommand() != null) {
      DogLog.log("Subsystems/Intake/CurrentCommand", getCurrentCommand().getName());
    } else {
      DogLog.log("Subsystems/Intake/CurrentCommand", "None");
    }
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
