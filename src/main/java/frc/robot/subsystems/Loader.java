package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.LoaderConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Loader extends SubsystemBase {
  Kraken motor;

  public Loader() {
    motor = KrakenBuilder.create(LOADER_MOTOR_ID, CAN_BUS, "Loader", "Loader Motor")
      .withCurrentLimit(80)
      .withIdleMode( NeutralModeValue.Brake)
      .withSlot0PID(0.5, 0, 0.00000001)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();
  }

  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command set(double speed) {
  return run(() -> motor.setControl(new VelocityVoltage(speed).withEnableFOC(true)));

  }

  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command set(DoubleSupplier speed) {
    return run(() ->  motor.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true)));
  }

  @Override
  public void periodic() {}
}
