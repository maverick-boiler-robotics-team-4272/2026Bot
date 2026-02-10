package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;
import static frc.robot.constants.FieldConstants.*;
import static frc.robot.subsystems.CommandSwerveDrivetrain.ROBOT_POSE;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

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

  public Command rev(double revSpeed) {
    return run(() -> motor.set(revSpeed));//setControl(new DutyCycleOut(revSpeed).withEnableFOC(true));
  }

  public double getRotationAngleRad() {
     return Math.atan(((2*FUEL_MAX_HEIGHT_M)/(ROBOT_POSE.getTranslation().getDistance(HUB_LOCATION))) * (1 + Math.sqrt(1 - (HUB_HEIGHT_M - SHOOTER_HEIGHT_M) / FUEL_MAX_HEIGHT_M)));
  }

  public double getRotationAngleDeg() {
    return getRotationAngleRad() * 180 / Math.PI;
  }

  public double getVelocity() {
    return Math.sqrt(2 * 9.81 * FUEL_MAX_HEIGHT_M) / Math.sin(getRotationAngleRad());
  }

  public double getVelocityRotPerSec() {
    return getVelocity() / SHOOTER_WHEEL_CIRCUMFERENCE;
  }

  public Command set(double speed) {
    return run(() -> motor.setControl(new VelocityVoltage(speed).withEnableFOC(true)));
  }
  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command set(DoubleSupplier speed) {
    DogLog.log(SHOOTER_LOG_KEY + "RPS", speed.getAsDouble());

    return run(() -> motor.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true)));
  }

  @Override
  public void periodic() {}
}
