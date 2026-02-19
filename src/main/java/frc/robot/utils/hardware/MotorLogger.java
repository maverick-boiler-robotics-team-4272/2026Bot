package frc.robot.utils.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import dev.doglog.DogLog;

public class MotorLogger {
  /**
   * Automatic motor logger
   *
   * @param subsystem is the name of the subsystem, the directory is taken care of from there
   * @param motors is a list of as many motors as you please
   */
  public static void log(String subsystem, Kraken motor) {
    StringBuilder keyBuilder =
        new StringBuilder("Subsystems/").append(subsystem).append("/Motors/");
    var supplyCurrent = motor.getSupplyCurrent();
    var statorCurrent = motor.getStatorCurrent();
    var voltage = motor.getMotorVoltage();
    var temp = motor.getDeviceTemp();
    var velocity = motor.getVelocity();
    var position = motor.getPosition();

    BaseStatusSignal.refreshAll(supplyCurrent, statorCurrent, voltage, temp, velocity, position);
    
    int deadLength = keyBuilder.length();
    keyBuilder.append(motor.getName()).append("/");
    int length = keyBuilder.length();

    keyBuilder.append("Supply Current");
    DogLog.log(keyBuilder.toString(), supplyCurrent.getValueAsDouble());

    keyBuilder.setLength(length);
    keyBuilder.append("Stator Current");
    DogLog.log(keyBuilder.toString(), statorCurrent.getValueAsDouble());

    keyBuilder.setLength(length);
    keyBuilder.append("Voltage");
    DogLog.log(keyBuilder.toString(), voltage.getValueAsDouble());

    keyBuilder.setLength(length);
    keyBuilder.append("Temperature");
    DogLog.log(keyBuilder.toString(), temp.getValueAsDouble());

    keyBuilder.setLength(length);
    keyBuilder.append("Velocity");
    DogLog.log(keyBuilder.toString(), velocity.getValueAsDouble());

    keyBuilder.setLength(length);
    keyBuilder.append("Position");
    DogLog.log(keyBuilder.toString(), position.getValueAsDouble());
    keyBuilder.setLength(deadLength);
  }
}
