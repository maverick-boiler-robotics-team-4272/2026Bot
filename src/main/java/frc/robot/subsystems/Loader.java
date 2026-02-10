package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.LoaderConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

public class Loader extends SubsystemBase {
  Kraken motor1;
  Kraken motor2;

  public Loader() {
    motor1 = KrakenBuilder.create(LOADER_MOTOR_1_ID, CAN_BUS, "Loader", "Loader Motor 1")
      .withCurrentLimit(80)
      .withIdleMode( NeutralModeValue.Brake)
      .withSlot0PID(0.5, 0, 0.00000001)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();
    motor2 = KrakenBuilder.create(LOADER_MOTOR_2_ID, CAN_BUS, "Loader", "Loader Motor 2")
      .withCurrentLimit(80)
      .withIdleMode( NeutralModeValue.Brake)
      .withSlot0PID(0.5, 0, 0.00000001)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();
  }

  
  public Command set1(double speed) {
  return run(() -> motor1.setControl(new VelocityVoltage(speed).withEnableFOC(true)));
  }
  public Command set2(double speed) {
    return run(() -> motor2.setControl(new VelocityVoltage(speed).withEnableFOC(true)));
  }
  public Command setBoth(double speed) {
    return run(() -> {
      new SequentialCommandGroup(set1(speed).withTimeout(.1), set2(speed).withTimeout(.1));
      
    });
  }

  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command set1(DoubleSupplier speed) {
    DogLog.log(LOADER_MOTOR_1_ID_LOG_KEY + "RPS", speed.getAsDouble());
    
    return run(() -> motor1.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true)));
    
  }
  public Command set2(DoubleSupplier speed) {
    DogLog.log(LOADER_MOTOR_2_ID_LOG_KEY + "RPS", speed.getAsDouble());

    return run(() -> motor2.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true)));
  }

  @Override
  public void periodic() {}
}
