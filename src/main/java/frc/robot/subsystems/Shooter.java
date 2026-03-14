package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

import java.security.PublicKey;
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
    shooterMotorLeft = KrakenBuilder.create(SHOOTER_MOTOR_LEFT_ID, "rio", "Shooter", "Shooter Motor Left")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(0.4, 0.0, 0.00, 0.35, 0, 0, 0.123)
        .withInversion(InvertedValue.Clockwise_Positive)
        .build(); // p = 0.1, d = 0.001, s = 0.2

    shooterMotorRight = KrakenBuilder.create(SHOOTER_MOTOR_RIGHT_ID, "rio", "Shooter", "Shooter Motor Right")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(0.4, 0.0, 0.00, 0.32, 0, 0, 0.123)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();

    hoodedMotor = KrakenBuilder.create(HOODED_MOTOR_ID, CAN_BUS, "Shooter", "Hooder Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(400, 0.0, 0.00, 0.0, 0.5, 0, 0)
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

  public Command setShooterState(DoubleSupplier anglePosition, DoubleSupplier rotationsPerSecond) {
    return run(
        () -> {
          desiredAngle = anglePosition.getAsDouble();
          hoodedMotor.setControl(
              new PositionVoltage(MathUtil.clamp(anglePosition.getAsDouble() + .000, 0, 0.085)).withEnableFOC(true));
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
              .setControl(new PositionVoltage(MathUtil.clamp(anglePosition + .000, 0, 0.085)).withEnableFOC(true));
          desiredSpeed = rotationsPerSecond;
          shooterMotorLeft.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
          shooterMotorRight.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
        });
  }

  public boolean isAtDesiredSpeed() {
    if (shooterMotorLeft.getVelocity().getValueAsDouble() >= desiredSpeed) {
      return true;
    }
    return false; // sim is always at the right velocity
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
  }
    
  public boolean isAtDesiredAngle() {
    if (hoodedMotor.getPosition().getValueAsDouble() > desiredAngle-0.001 && 
      hoodedMotor.getPosition().getValueAsDouble() < desiredAngle+0.001) {
      return true;
    }
    return false; 
  }
}