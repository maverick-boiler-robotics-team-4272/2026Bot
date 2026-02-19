package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.LoaderConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import java.util.function.DoubleSupplier;

public class Loader extends SubsystemBase {
  Kraken motor1;
  Kraken motor2;

  public Loader() {
    motor1 =
        KrakenBuilder.create(LOADER_MOTOR_1_ID, CAN_BUS, "Loader", "Loader Motor 1")
            .withCurrentLimit(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true))
            .withIdleMode(NeutralModeValue.Coast)
            .withSlot0PID(0.5, 0, 0.00000001)
            .withInversion(InvertedValue.CounterClockwise_Positive)
            .build();
    motor2 =
        KrakenBuilder.create(LOADER_MOTOR_2_ID, CAN_BUS, "Loader", "Loader Motor 2")
            .withCurrentLimit(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true))
            .withIdleMode(NeutralModeValue.Coast)
            .withSlot0PID(0.5, 0, 0.00000001)
            .withInversion(InvertedValue.CounterClockwise_Positive)
            .build();
  }

  public Command loadLeft(double speed) {
    return run(() -> motor1.setControl(new VelocityVoltage(speed).withEnableFOC(true)));
  }

  public Command loadRight(double speed) {
    return run(() -> motor2.setControl(new VelocityVoltage(speed).withEnableFOC(true)));
  }

  public Command loadBoth(double speed) {
    return run(
        () -> {
          motor1.setControl(new VelocityVoltage(speed).withEnableFOC(true));
          motor2.setControl(new VelocityVoltage(speed).withEnableFOC(true));
        });
  }

  /**
   * @param speed in rotations per second
   * @return
   */
  public Command loadLeft(DoubleSupplier speed) {
    return run(
        () -> motor1.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true)));
  }

  public Command loadRight(DoubleSupplier speed) {
    return run(
        () -> motor2.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true)));
  }

  @Override
  public void periodic() {}
}
