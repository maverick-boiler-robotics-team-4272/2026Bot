package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.*;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
  Kraken motor;
  Kraken motor2;

  double speed;
  double rotations;

  public Intake() {
    motor = KrakenBuilder.create(INTAKE_MOTOR_ID, CAN_BUS, "Intake", "Intake Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PID(5, 0, 0)
        .build();
    motor2 = KrakenBuilder.create(INTAKE_MOTOR_2_ID, CAN_BUS, "Intake", "Actuation Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Brake)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PID(5, 0, 0)
        .build();
  }

  /**
   * @param speed in rotations per second
   * @return
   */
  public Command intake(double speed) {
    return run(() -> {
      this.speed = speed;
      motor.setControl(
        new VelocityVoltage(speed).withEnableFOC(true));});
  }

  /**
   * @param speed in rotations per second
   * @return
   */
  public Command intake(DoubleSupplier speed) {
    return run(
        () -> {
          this.speed = speed.getAsDouble();
          motor.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true));});
  }

  /**
   * @param distance in inches of the actuation distance
   * @return
   */
  public Command extend(double rotations) {
    this.rotations = rotations;
    return run(() -> motor2.setControl(new PositionVoltage(rotations).withEnableFOC(true)));
  }

  /**
   * @param distance in inches of the actuation distance as a supplier
   * @return
   */
  public Command extend(DoubleSupplier rotations) {
    this.rotations = rotations.getAsDouble();
    return run(() -> motor2.setControl(new PositionVoltage(this.rotations).withEnableFOC(true)));
  }

  public Command setIntakeState(double rotationsDistance, double rotationsPerSecond) {
    return run(
        () -> {
          rotations = rotationsDistance;
          speed = rotationsPerSecond;
          motor2.setControl(new PositionVoltage(rotations).withEnableFOC(true));
          motor.setControl(new VelocityVoltage(rotationsPerSecond).withEnableFOC(true));
        });
  }

  public Command defaultCommand() {
    return run(
        () -> {
          setIntakeState(rotations, 0);
        });
  }

  public Command zeroHood() {
    return run(
      () -> {
        motor2.setControl(new VoltageOut(1).withEnableFOC(true));
        motor2.setPosition(motor2.getPosition().getValue());
      }
    );
  }

  @Override
  public void periodic() {
    DogLog.log(INTAKE_KEY + "distance", rotations);
    DogLog.log(INTAKE_KEY + "speed", speed);
  }
}
