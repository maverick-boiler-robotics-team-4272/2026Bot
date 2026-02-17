package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;
import static frc.robot.constants.SubsystemConstants.*;

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

public class Intake extends SubsystemBase {
  Kraken motor;
  Kraken motor2;

  public Intake() {
    motor = KrakenBuilder.create(INTAKE_MOTOR_ID, CAN_BUS, "Intake", "Intake Motor")
        .withCurrentLimit(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PID(5, 0, 0)
        .build();
    motor2 = KrakenBuilder.create(INTAKE_MOTOR_2_ID, CAN_BUS, "Intake", "Actuation Motor")
        .withCurrentLimit(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Brake)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PID(5, 0, 0)
        .build();
  }

  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command intake(double speed) {
    return run(() -> motor.setControl(new VelocityVoltage(speed).withEnableFOC(true)));
  }

  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command intake(DoubleSupplier speed) {
    return run(() -> motor.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true)));
  }

  /**
   * 
   * @param distance in inches of the actuation distance
   * @return
   */
  public Command extend(double distance) {
    double rotations = distance / Math.PI;
    return run(() -> motor2.setControl(new PositionVoltage(rotations).withEnableFOC(true)));
  }

  /**
   * 
   * @param distance in inches of the actuation distance as a supplier
   * @return
   */
  public Command extend(DoubleSupplier distance) {
    double rotations = distance.getAsDouble() / Math.PI;
    return run(() -> motor2.setControl(new PositionVoltage(rotations).withEnableFOC(true)));
  }

  public Command defaultCommand() {
    return run(
        () -> {
          extend(0);
          intake(0);
        });
  }

  @Override
  public void periodic() {
  }
}
