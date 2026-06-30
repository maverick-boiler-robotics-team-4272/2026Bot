package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  Kraken shooterMotorLeft;
  Kraken shooterMotorRight;
  Kraken hoodedMotor;

  double desiredSpeed = 0;
  double desiredAngle = 0;

  public Shooter() {
    // PID needs to be aggressive
    VoltageConfigs volts = new VoltageConfigs().withPeakForwardVoltage(12);

    shooterMotorLeft = KrakenBuilder.create(SHOOTER_MOTOR_LEFT_ID, "rio", "Shooter", "Shooter Motor Left")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(60)
                .withSupplyCurrentLowerTime(2.0)
                .withStatorCurrentLimitEnable(false))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(0.2/* 5 */, 0.0, 0.00, .51, 0, 0, 0.123 * 18.0 / 24.0)
        .withInversion(InvertedValue.Clockwise_Positive)
        .build();
    shooterMotorRight = KrakenBuilder.create(SHOOTER_MOTOR_RIGHT_ID, "rio", "Shooter", "Shooter Motor Right")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(60)
                .withSupplyCurrentLowerTime(2.0)
                .withStatorCurrentLimitEnable(false))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(0.2/* 5 */, 0.0, 0.00, .48, 0, 0, 0.123 * 18.0 / 24.0)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();
    shooterMotorLeft.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(18.0 / 24.0));
    shooterMotorRight.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(18.0 / 24.0));

        shooterMotorRight.getConfigurator().apply(volts);

    hoodedMotor = KrakenBuilder.create(HOODED_MOTOR_ID, CAN_BUS, "Shooter", "Hooder Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(100, 0.0, 0.00, 0.0, 0.5, 0, 0)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();

    hoodedMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(348.0 * 24.0 / (16.0 * 11.0)));
  }

  public Command zeroHood() {
    return run(
        () -> {
          desiredAngle = 0.0;
          hoodedMotor.setControl(new VoltageOut(-1).withEnableFOC(true));
          hoodedMotor.setPosition(0.0);
          shooterMotorLeft.setControl(new VoltageOut(0).withEnableFOC(true));
          shooterMotorRight.setControl(new VoltageOut(0).withEnableFOC(true));
        });
  }

  public Command oof() {
    return run(
      () -> {
        hoodedMotor.setControl(new VoltageOut(1).withEnableFOC(true));
        shooterMotorLeft.setControl(new VoltageOut(0).withEnableFOC(true));
        shooterMotorRight.setControl(new VoltageOut(0).withEnableFOC(true));
      }
    );
  }
  public Command setShooterState(DoubleSupplier anglePosition, DoubleSupplier rotationsPerSecond) {
    return run(
        () -> {
          desiredAngle = anglePosition.getAsDouble();
          hoodedMotor.setControl(
              new PositionVoltage(MathUtil.clamp(anglePosition.getAsDouble() + .000, 0, 0.105)).withEnableFOC(true));
          desiredSpeed = rotationsPerSecond.getAsDouble();
          shooterMotorLeft.setControl(new VelocityVoltage(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
          shooterMotorRight.setControl(new VelocityVoltage(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
        });
  }

  public Command setShooterState(double anglePosition, double rotationsPerSecond) {
    return run(
        () -> {
          desiredAngle = anglePosition;
          hoodedMotor
              .setControl(new PositionVoltage(MathUtil.clamp(anglePosition + .000, 0, 0.105)).withEnableFOC(true));
          desiredSpeed = rotationsPerSecond;
          shooterMotorLeft.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
          shooterMotorRight.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
        });
  }

  public Command defaultCommand() {
    return run(
        () -> {
          desiredAngle = 0;
          desiredSpeed = 0;
          hoodedMotor.setControl(new PositionVoltage(0).withEnableFOC(true));
          shooterMotorLeft.setControl(new VoltageOut(0).withEnableFOC(true));
          shooterMotorRight.setControl(new VoltageOut(0).withEnableFOC(true));
        });
  }

  public Command setVoltage(DoubleSupplier anglePosition, DoubleSupplier rotationsPerSecond) {
    return run(
        () -> {
          desiredAngle = anglePosition.getAsDouble();
          hoodedMotor.setControl(
              new VoltageOut(anglePosition.getAsDouble()).withEnableFOC(true));
          desiredSpeed = rotationsPerSecond.getAsDouble();
          shooterMotorLeft.setControl(new VoltageOut(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
          shooterMotorRight.setControl(new VoltageOut(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
        });
  }

  @Override
  public void periodic() {
    DogLog.log(SHOOTER_LOG_KEY + "/recomended speed", desiredSpeed);
    DogLog.log(SHOOTER_LOG_KEY + "/recomended rotations", desiredAngle);
    DogLog.log(SHOOTER_LOG_KEY + "/Checks/speed", isAtDesiredSpeedCheck().getAsBoolean());
    DogLog.log(SHOOTER_LOG_KEY + "/Checks/angle", isAtDesiredAngleCheck().getAsBoolean());
  }
    
  public boolean isAtDesiredAngle() {
    if (hoodedMotor.getPosition(false).getValueAsDouble() > desiredAngle - 0.001 &&
        hoodedMotor.getPosition(false).getValueAsDouble() < desiredAngle + 0.001) {
      return true;
    }
    return false; 
  }

  public boolean isAtDesiredSpeed() {
    if (shooterMotorLeft.getVelocity(false).getValueAsDouble() >= desiredSpeed - 1
        || shooterMotorRight.getVelocity(false).getValueAsDouble() >= desiredSpeed - 1) {
      return true;
    }
    return false;
  }

  public BooleanSupplier isAtDesiredSpeedCheck() {
    return () -> isAtDesiredSpeed();
  }

  public BooleanSupplier isAtDesiredAngleCheck() {
    return () -> isAtDesiredAngle();
  }
}