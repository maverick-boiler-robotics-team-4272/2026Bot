package frc.robot.subsystems;

import static frc.robot.constants.FieldConstants.HUB_LOCATION;
import static frc.robot.constants.SubsystemConstants.CAN_BUS;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;
import static frc.robot.subsystems.CommandSwerveDrivetrain.ROBOT_POSE;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Kraken;
import frc.robot.utils.hardware.KrakenBuilder;

public class Shooter extends SubsystemBase {
  Kraken shooterMotorLeft;
  Kraken shooterMotorRight;
  Kraken hoodedMotor;

  public Shooter() {
    shooterMotorLeft = KrakenBuilder.create(SHOOTER_MOTOR_LEFT_ID, CAN_BUS, "Shooter", "Shooter Motor Left")
      .withCurrentLimit(80)
      .withIdleMode(NeutralModeValue.Coast)
      .withSlot0PID(0.6, 0, 0.000000001)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();

    shooterMotorRight = KrakenBuilder.create(SHOOTER_MOTOR_RIGHT_ID, CAN_BUS, "Shooter", "Shooter Motor Right")
      .withCurrentLimit(80)
      .withIdleMode(NeutralModeValue.Coast)
      .withSlot0PID(0.6, 0, 0.000000001)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();
    hoodedMotor = KrakenBuilder.create(HOODED_MOTOR_ID, CAN_BUS, "Shooter", "Hooder Motor")
      .withCurrentLimit(40)
      .withIdleMode(NeutralModeValue.Coast)
      .withSlot0PID(5, 0, 0)
      .withInversion(InvertedValue.CounterClockwise_Positive)
      .build();
  }
  
  public Command setAngle(DoubleSupplier angle) {
    return run(
      () -> hoodedMotor.setControl(new PositionVoltage(angle.getAsDouble()).withEnableFOC(true))
    );
  }

  public Command setDesiredAngle() {
    return run(
      () -> hoodedMotor.setControl(new PositionVoltage(ANGLE_LOOKUP.get(ROBOT_POSE.getTranslation().getDistance(HUB_LOCATION))).withEnableFOC(true))
    );
  }
  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command set(double speed) {
    return run(() -> {
      shooterMotorLeft.setControl(new VelocityVoltage(speed).withEnableFOC(true));
      shooterMotorRight.setControl(new VelocityVoltage(speed).withEnableFOC(true));
      }
    );
  }
  /**
   * 
   * @param speed in rotations per second
   * @return
   */
  public Command set(DoubleSupplier speed) {
    DogLog.log(SHOOTER_LOG_KEY + "RPS", speed.getAsDouble());

    return run(() -> {
      shooterMotorLeft.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true));
      shooterMotorRight.setControl(new VelocityVoltage(speed.getAsDouble()).withEnableFOC(true));
      }
    );
  }

  @Override
  public void periodic() {}
}
