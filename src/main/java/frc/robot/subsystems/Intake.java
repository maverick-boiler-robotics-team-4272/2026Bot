package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;
import static frc.robot.constants.SubsystemConstants.*;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import frc.robot.utils.hardware.MotorLogger;

public class Intake extends SubsystemBase {
  Kraken motor;
  
  public Intake() {
    motor = KrakenBuilder.create(INTAKE_MOTOR_ID, CAN_BUS, "Intake", "Intake Motor")
      .withCurrentLimit(80)
      .withIdleMode(NeutralModeValue.Coast)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();
  }

public Runnable intake(double speed) {
  return () -> motor.set(speed);//new DutyCycleOut(speed).withEnableFOC(true);
} 

public Runnable intake(DoubleSupplier speed) {
  return () -> motor.set(speed.getAsDouble());//new DutyCycleOut(speed).withEnableFOC(true);
} 

  @Override
  public void periodic() {}
}
