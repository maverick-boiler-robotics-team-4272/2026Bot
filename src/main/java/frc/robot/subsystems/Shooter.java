package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;
import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.*;
import static frc.robot.constants.FieldConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import frc.robot.utils.hardware.MotorLogger;

public class Shooter extends SubsystemBase {
  Kraken motor;

  public Shooter() {
    motor = KrakenBuilder.create(SHOOTER_MOTOR_ID, CAN_BUS, "Shooter", "Shooter Motor")
      .withCurrentLimit(80)
      .withIdleMode(NeutralModeValue.Coast)
      .withSlot0PID(0.6, 0, 0.000000001)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();
  }

  public Runnable rev(double revSpeed) {
    return () -> motor.set(revSpeed);//setControl(new DutyCycleOut(revSpeed).withEnableFOC(true));
  }

  double rotationAngle;
  public void getRotationAngle() {
    rotationAngle = Math.atan(((2*FUEL_MAX_HEIGHT_M)/(DISTANCE_TO_HUB_M)) * (1 + Math.sqrt(1 - (HUB_HEIGHT_M - SHOOTER_HEIGHT_M) / FUEL_MAX_HEIGHT_M)));
  }

  double velocity;
  public void getVelocity() {
    velocity = Math.sqrt(2 * 9.81 * FUEL_MAX_HEIGHT_M) / Math.sin(rotationAngle);
  }

  public Runnable rev(DoubleSupplier revSpeed) {//, double p, double i, double d) {
    DogLog.log(SHOOTER_LOG_KEY + "RPS", revSpeed.getAsDouble());

    return () -> motor.setControl(new VelocityVoltage(revSpeed.getAsDouble()).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    MotorLogger.log("Shooter", motor);
  }
}
