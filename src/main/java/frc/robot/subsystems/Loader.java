package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.LoaderConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Loader extends SubsystemBase {
  Kraken motor1;
  Kraken motor2;

  double desiredSpeed = 0;

  Timer loadLeftTimer;
  Timer loadRightTimer;

  boolean leftJam;
  boolean rightJam;

  public Loader() {
    motor1 = KrakenBuilder.create(LOADER_MOTOR_1_ID, CAN_BUS, "Loader", "Loader Motor 1") // right
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(0.0, 0, 0.0, 0, 0, 0, 0.12413)
        .withInversion(InvertedValue.Clockwise_Positive)
        .build();
    motor1.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1));

    motor2 = KrakenBuilder.create(LOADER_MOTOR_2_ID, CAN_BUS, "Loader", "Loader Motor 2") // Left
        .withCurrentLimit(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true))
        .withIdleMode(NeutralModeValue.Coast)
        .withSlot0PIDSGAV(0.0, 0, 0.0, 0, 0, 0, 0.12413)
        .withInversion(InvertedValue.CounterClockwise_Positive)
        .build();
    motor2.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1));

    loadLeftTimer = new Timer();
    loadRightTimer = new Timer();

    leftJam = false;
    rightJam = false;
  }

  public Command loadBoth(double speed) {
    return run(
        () -> {
          this.desiredSpeed = speed;
          // loadLeftTimer.start();
          // loadRightTimer.start();
          // if (loadLeftTimer.get() < 2) {
          //   leftJam = false;
          // } else if (motor2.getSupplyCurrent().getValueAsDouble() > 14) {
          //   loadLeftTimer.restart();
          //   leftJam = false;
          // } else {
          //   leftJam = true;
          // }

          // if (loadRightTimer.get() < 2) {
          //   rightJam = false;
          // } else if (motor1.getSupplyCurrent().getValueAsDouble() > 14) {
          //   loadRightTimer.restart();
          //   rightJam = false;
          // } else {
          //   rightJam = true;
          // }

          motor1.setControl(new VelocityVoltage(speed).withEnableFOC(true));
          motor2.setControl(new VelocityVoltage(speed).withEnableFOC(true));
        });
        // .until(isJammed()).finallyDo(() -> {
        //   loadLeftTimer.stop();
        //   loadRightTimer.stop();
        //   loadLeftTimer.reset();
        //   loadRightTimer.reset();
        // });
  }

  public BooleanSupplier isJammed() {
    return () -> false;
  }

  public Command dejam() {
    return run(() -> {
      motor1.setControl(new VelocityVoltage(-40).withEnableFOC(true));
      motor2.setControl(new VelocityVoltage(-40).withEnableFOC(true));
    }).withTimeout(0.2).finallyDo(() -> {
      leftJam = false;
      rightJam = false;
      loadLeftTimer.stop();
      loadRightTimer.stop();
      loadLeftTimer.reset();
      loadRightTimer.reset();
    });
  }

  @Override
  public void periodic() {
    DogLog.log(LOADER_KEY + "Speed", desiredSpeed);
    // DogLog.log(LOADER_KEY + "Jammed", isJammed().getAsBoolean());
  }
}
