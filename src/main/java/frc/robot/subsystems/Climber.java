package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.ClimberConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {
  Kraken motor;

  public Climber() {
    motor = KrakenBuilder.create(CLIMBER_MOTOR_ID, CAN_BUS, "Climber", "Climber Motor")
        .withCurrentLimit(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Brake)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .withSlot0PID(0, 0, 0)
        .build();
  }

  public Command climb(double rotations) {
    return run(
        () -> motor.setControl(new PositionVoltage(rotations).withEnableFOC(true)));
  }

  @Override
  public void periodic() {
  }
}
