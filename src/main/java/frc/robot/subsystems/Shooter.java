package frc.robot.subsystems;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;
import static frc.robot.subsystems.CommandSwerveDrivetrain.ROBOT_POSE;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

public class Shooter extends SubsystemBase {
  Kraken shooterMotorLeft;
  Kraken shooterMotorRight;
  Kraken hoodedMotor;

  double desiredSpeed = 0;

  public Shooter() {
    // PID needs to be aggressive
    shooterMotorLeft = KrakenBuilder.create(SHOOTER_MOTOR_LEFT_ID, CAN_BUS, "Shooter", "Shooter Motor Left")
        .withCurrentLimit(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PID(0.6, 0, 0.000000001)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();

    shooterMotorRight = KrakenBuilder.create(SHOOTER_MOTOR_RIGHT_ID, CAN_BUS, "Shooter", "Shooter Motor Right")
        .withCurrentLimit(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PID(0.6, 0, 0.000000001)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();

    hoodedMotor = KrakenBuilder.create(HOODED_MOTOR_ID, CAN_BUS, "Shooter", "Hooder Motor")
        .withCurrentLimit(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PID(5, 0, 0)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();
  }

  public Command setAngle(DoubleSupplier angle) {
    return run(
        () -> hoodedMotor.setControl(new PositionVoltage(angle.getAsDouble()).withEnableFOC(true)));
  }

  public Command setAngle(double angle) {
    return run(
        () -> hoodedMotor.setControl(new PositionVoltage(angle).withEnableFOC(true)));
  }

  public Command setDesiredAngle() {
    if (ROBOT_POSE.getX() < 4.03 || ROBOT_POSE.getX() > FIELD_LENGTH_M - 4.03) {
      return setAngle(ANGLE_LOOKUP.get(ROBOT_POSE.getTranslation().getDistance(HUB_LOCATION)));
    } else {
      return setAngle(40); // maximize distance! along with the fact that the shooter can only go that far
    }
  }

  public Command setDesiredSpeed() {
    if (ROBOT_POSE.getX() < 4.03 || ROBOT_POSE.getX() > FIELD_LENGTH_M - 4.03) {
      return rev(VELOCITY_LOOKUP.get(ROBOT_POSE.getTranslation().getDistance(HUB_LOCATION)));
    } else {
      return rev(45); // Is this a good shuttle speed?
    }
  }

  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command rev(double speed) {
    desiredSpeed = speed;
    return run(() -> {
      shooterMotorLeft.setControl(new VelocityVoltage(speed).withEnableFOC(true));
      shooterMotorRight.setControl(new VelocityVoltage(speed).withEnableFOC(true));
    });
  }

  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command rev(DoubleSupplier speed) {
    return run(() -> {
      shooterMotorLeft.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true));
      shooterMotorRight.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true));
    });
  }

  public boolean isAtDesiredSpeed() {
    if (shooterMotorLeft.getRotorVelocity().getValueAsDouble() >= desiredSpeed) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
  }
}
