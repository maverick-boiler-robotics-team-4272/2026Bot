package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  Kraken shooterMotorLeft;
  Kraken shooterMotorRight;
  Kraken hoodedMotor;

  double desiredSpeed = 0;
  double desiredAngle = 0;

  public Shooter() {
    // PID needs to be aggressive
    shooterMotorLeft = KrakenBuilder.create(SHOOTER_MOTOR_LEFT_ID, CAN_BUS, "Shooter", "Shooter Motor Left")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PID(0.6, 0, 0.000000001)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();

    shooterMotorRight = KrakenBuilder.create(SHOOTER_MOTOR_RIGHT_ID, CAN_BUS, "Shooter", "Shooter Motor Right")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PID(0.6, 0, 0.000000001)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();

    hoodedMotor = KrakenBuilder.create(HOODED_MOTOR_ID, CAN_BUS, "Shooter", "Hooder Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PID(5, 0, 0)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();
  }

  public Command setAngle(DoubleSupplier angle) {
    return run(
        () -> {
          hoodedMotor.setControl(new PositionVoltage(angle.getAsDouble()).withEnableFOC(true));
          desiredAngle = angle.getAsDouble();
        });
  }

  public Command setAngle(double angle) {
    return run(() -> {
      hoodedMotor.setControl(new PositionVoltage(angle).withEnableFOC(true));
      desiredAngle = angle;
    });
  }

  /**
   * @param speed in rotations per second
   * @return
   */
  public Command rev(double speed) {
    return run(
        () -> {
          desiredSpeed = speed;
          shooterMotorLeft.setControl(new VelocityVoltage(speed).withEnableFOC(true));
          shooterMotorRight.setControl(new VelocityVoltage(speed).withEnableFOC(true));
        });
  }

  /**
   * @param speed in rotations per second
   * @return
   */
  public Command rev(DoubleSupplier speed) {
    return run(
        () -> {
          desiredSpeed = speed.getAsDouble();
          shooterMotorLeft.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true));
          shooterMotorRight.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true));
        });
  }

  public Command setShooterState(DoubleSupplier anglePosition, DoubleSupplier rotationsPerSecond) {
    return run(
        () -> {
          desiredAngle = anglePosition.getAsDouble();
          hoodedMotor.setControl(new PositionVoltage(anglePosition.getAsDouble()).withEnableFOC(true));
          desiredSpeed = rotationsPerSecond.getAsDouble();
          shooterMotorLeft.setControl(new VelocityVoltage(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
          shooterMotorRight.setControl(new VelocityVoltage(rotationsPerSecond.getAsDouble()).withEnableFOC(true));
        });
  }

  public Command setShooterState(double anglePosition, double rotationsPerSecond) {
    return run(
        () -> {
          desiredAngle = anglePosition;
          hoodedMotor.setControl(new PositionVoltage(anglePosition).withEnableFOC(true));
          desiredSpeed = rotationsPerSecond;
          shooterMotorLeft.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
          shooterMotorRight.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
        });
  }

  public boolean isAtDesiredSpeed() {
    if (shooterMotorLeft.getVelocity().getValueAsDouble() >= desiredSpeed - 1) {
      return true;
    }
    return Robot.isReal() ? false : true; // sim is always at the right velocity
  }

  @Override
  public void periodic() {
    DogLog.log(SHOOTER_LOG_KEY + "/recomended speed", desiredSpeed);
    DogLog.log(SHOOTER_LOG_KEY + "/recomended angle", desiredAngle);
  }
}
