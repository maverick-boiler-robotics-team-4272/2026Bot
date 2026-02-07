package frc.robot.utils.hardware;

import com.ctre.phoenix6.BaseStatusSignal;

import dev.doglog.DogLog;
import frc.robot.Robot;

public class MotorLogger{
    /**
     * Automatic motor logger
     * @param subsystem is the name of the subsystem, the directory is taken care of from there
     * @param motors is a list of as many motors as you please
     */
    public static void log(String subsystem, Kraken motor) {
        StringBuilder keyBuilder = new StringBuilder("Subsystems/").append(subsystem).append("/Motors/");
        var current = motor.getSupplyCurrent();
        var voltage = motor.getMotorVoltage();
        var temp = motor.getDeviceTemp();
        var velocity = motor.getVelocity();
        BaseStatusSignal.refreshAll(
            current,
            voltage,
            temp,
            velocity
        );
        int deadLength = keyBuilder.length();
        keyBuilder.append(motor.getName()).append("/");
        int length = keyBuilder.length();

        keyBuilder.append("Current");
        DogLog.log(keyBuilder.toString(), current.getValueAsDouble());

        keyBuilder.setLength(length);
        keyBuilder.append("Voltage");
        DogLog.log(keyBuilder.toString(), voltage.getValueAsDouble());

        keyBuilder.setLength(length);
        keyBuilder.append("Temperature");
        DogLog.log(keyBuilder.toString(),temp.getValueAsDouble());

        keyBuilder.setLength(length);
        keyBuilder.append("Velocity");
        DogLog.log(keyBuilder.toString(), velocity.getValueAsDouble());
        keyBuilder.setLength(deadLength);   
    }   
}
