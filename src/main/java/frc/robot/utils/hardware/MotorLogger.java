package frc.robot.utils.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class MotorLogger {
  static StatusSignal<Current> supplyCurrent;
  static StatusSignal<Current> statorCurrent;
  static StatusSignal<Voltage> voltage;
  static StatusSignal<AngularVelocity> velocity;
  static StatusSignal<Angle> position;
  /**
   * Automatic motor logger
   *
   * @param subsystem is the name of the subsystem, the directory is taken care of
   *                  from there
   * @param motors    is a list of as many motors as you please
   */
  
  public static void log(String subsystem, Kraken motor) {
    StringBuilder keyBuilder = new StringBuilder("Subsystems/").append(subsystem).append("/Motors/");
    supplyCurrent = motor.getSupplyCurrent(false);
    statorCurrent = motor.getStatorCurrent(false);
    voltage = motor.getMotorVoltage(false);
    velocity = motor.getVelocity(false);
    position = motor.getPosition(false);

    BaseStatusSignal.refreshAll(supplyCurrent, statorCurrent, voltage, velocity, position);

    int deadLength = keyBuilder.length();
    keyBuilder.append(motor.getName()).append("/");
    int length = keyBuilder.length();

    keyBuilder.append("Supply Current");
    DogLog.log(keyBuilder.toString(), supplyCurrent.getValueAsDouble());

    keyBuilder.setLength(length);
    keyBuilder.append("Stator Current");
    DogLog.log(keyBuilder.toString(), statorCurrent.getValueAsDouble());

    // keyBuilder.setLength(length);
    // keyBuilder.append("Voltage");
    // DogLog.log(keyBuilder.toString(), voltage.getValueAsDouble());

    keyBuilder.setLength(length);
    keyBuilder.append("Velocity");
    DogLog.log(keyBuilder.toString(), velocity.getValueAsDouble());

    keyBuilder.setLength(length);
    keyBuilder.append("Position");
    DogLog.log(keyBuilder.toString(), position.getValueAsDouble());
    keyBuilder.setLength(deadLength);
  }

}
