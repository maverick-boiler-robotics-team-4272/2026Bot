package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import frc.robot.utils.hardware.MotorLogger;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.LoaderConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Loader extends SubsystemBase {
  Kraken motor;

  public Loader() {
    motor = KrakenBuilder.create(LOADER_MOTOR_ID, CAN_BUS, "Loader Motor")
      .withCurrentLimit(80)
      .withIdleMode( NeutralModeValue.Brake)
      .withSlot0PID(0.5, 0, 0.00000001)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();
  }

  public Runnable load(double loadSpeed) {
    return () -> motor.set(loadSpeed);//setControl(new DutyCycleOut(loadSpeed).withEnableFOC(true));
  }

  public Runnable load(DoubleSupplier loadSpeed) {
    return () ->  motor.setControl(new VelocityVoltage(loadSpeed.getAsDouble()).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    MotorLogger.log("Loader", motor);
  }
}
