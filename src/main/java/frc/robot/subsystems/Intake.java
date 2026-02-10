package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;
import static frc.robot.constants.SubsystemConstants.*;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

public class Intake extends SubsystemBase {
  Kraken motor;
  
  public Intake() {
    motor = KrakenBuilder.create(INTAKE_MOTOR_ID, CAN_BUS, "Intake", "Intake Motor")
      .withCurrentLimit(80)
      .withIdleMode(NeutralModeValue.Coast)
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
    return run(() -> motor.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true)));
  } 

  @Override
  public void periodic() {}
}
