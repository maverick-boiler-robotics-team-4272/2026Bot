package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import frc.robot.utils.hardware.MotorLogger;

import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//Agitaor???
public class Hopper extends SubsystemBase {

  Kraken lowerMotor;
  Kraken lowerMotor2;
  Kraken upperMotor;

  double p = 2;
  double i = 0.0;
  double d = 0.0000000;

  public Hopper() {
    lowerMotor = KrakenBuilder.create(HOPPER_LOWER_MOTOR_ID, CAN_BUS, "Hopper", "Left Lower Motor")
      .withCurrentLimit(80)
      .withIdleMode(NeutralModeValue.Coast)
      .withSlot0PID(p,i,d)
      .withInversion(InvertedValue.Clockwise_Positive)
      .build();
    
    lowerMotor2 = KrakenBuilder.create(HOPPER_LOWER_MOTOR_2_ID, CAN_BUS, "Hopper", "Right Lower Motor")
      .withCurrentLimit(80)
      .withIdleMode(NeutralModeValue.Coast)
      .withSlot0PID(p,i,d)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();

    upperMotor = KrakenBuilder.create(HOPPER_UPPER_MOTOR_ID, CAN_BUS, "Hopper", "Upper Motor")
      .withCurrentLimit(80)
      .withIdleMode(NeutralModeValue.Coast)
      .withSlot0PID(0.4, 0, 0.00000000001)
      .withInversion(InvertedValue.Clockwise_Positive)
      .build();
  }

  public Runnable agitate(double agitateSpeed) {
    return () -> { 
      lowerMotor.set(agitateSpeed);
      lowerMotor2.set(agitateSpeed);
      upperMotor.set(agitateSpeed);
    };
  }

  public Runnable agitate(DoubleSupplier lowerAgitation, DoubleSupplier upperAgitation) {
    return () -> {
      lowerMotor.setControl(new VelocityVoltage(lowerAgitation.getAsDouble()).withEnableFOC(true));
      lowerMotor2.setControl(new VelocityVoltage(lowerAgitation.getAsDouble()).withEnableFOC(true));
      upperMotor.setControl(new VelocityVoltage(upperAgitation.getAsDouble()).withEnableFOC(true));
    };
  }

  @Override
  public void periodic() {}
}
