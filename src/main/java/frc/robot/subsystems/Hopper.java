package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.*;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;
import java.util.function.DoubleSupplier;

// Agitaor Hopper???
public class Hopper extends SubsystemBase {

  Kraken lowerMotor;
  Kraken lowerMotor2;
  Kraken upperMotor;

  double desiredUpperSpeed = 0;
  double desiredLowerSpeed = 0;

  public Hopper() {
    lowerMotor = KrakenBuilder.create(HOPPER_LOWER_MOTOR_ID, CAN_BUS, "Hopper", "Left Lower Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(false))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(10, 00, 0.1, 10, 0, 0, 01)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();

    lowerMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(24.0 / 11.0));

    lowerMotor2 = KrakenBuilder.create(HOPPER_LOWER_MOTOR_2_ID, CAN_BUS, "Hopper", "Right Lower Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(false))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(10, 00, 0.0, 10, 0, 0, 01)
        .withInversion(InvertedValue.Clockwise_Positive)
        .build();

    lowerMotor2.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(24.0 / 11.0));

    upperMotor = KrakenBuilder.create(HOPPER_UPPER_MOTOR_ID, CAN_BUS, "Hopper", "Upper Motor")
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                // .withSupplyCurrentLowerLimit(60)
                // .withSupplyCurrentLowerTime(3.0)
                .withStatorCurrentLimitEnable(false))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(0.4, 0, 0.0, 0, 0, 0, 0.12413 * 24.0 / 11.0)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();

    upperMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(24.0 / 11.0));
  }

  /**
   * @param lowerSpeed in rotations per second
   * @param upperSpeed in rotations per secod
   * @return
   */
  public Command agitate(double lowerSpeed, double upperSpeed) {
    return run(
        () -> {
          this.desiredUpperSpeed = upperSpeed;
          this.desiredLowerSpeed = lowerSpeed;
          lowerMotor.setControl(new VelocityVoltage(lowerSpeed).withEnableFOC(true));
          lowerMotor2.setControl(new VelocityVoltage(lowerSpeed).withEnableFOC(true));
          upperMotor.setControl(new VelocityVoltage(upperSpeed).withEnableFOC(true));
        });
  }

  /**
   * @param lowerSpeed in rotations per second
   * @param upperSpeed in rotations per secod
   * @return
   */
  public Command agitate(DoubleSupplier lowerSpeed, DoubleSupplier upperSpeed) {
    return run(
        () -> {
          this.desiredUpperSpeed = upperSpeed.getAsDouble();
          this.desiredLowerSpeed = lowerSpeed.getAsDouble();
          lowerMotor.setControl(new VelocityVoltage(lowerSpeed.getAsDouble()).withEnableFOC(true));
          lowerMotor2.setControl(new VelocityVoltage(lowerSpeed.getAsDouble()).withEnableFOC(true));
          upperMotor.setControl(new VelocityVoltage(upperSpeed.getAsDouble()).withEnableFOC(true));
        });
  }

  public Command stop() {
    return run(
        () -> {
          desiredLowerSpeed = 0;
          desiredUpperSpeed = 0;
          lowerMotor.setControl(new VoltageOut(0).withEnableFOC(true));
          lowerMotor2.setControl(new VoltageOut(0).withEnableFOC(true));
          upperMotor.setControl(new VoltageOut(0).withEnableFOC(true));
        });
  }

  @Override
  public void periodic() {
    DogLog.log(HOPPER_KEY + "upperSpeed", desiredUpperSpeed);
    DogLog.log(HOPPER_KEY + "lowerSpeed", desiredLowerSpeed);
  }
}
