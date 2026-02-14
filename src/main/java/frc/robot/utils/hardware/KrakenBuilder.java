package frc.robot.utils.hardware;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class KrakenBuilder {
    private Kraken motor;
    private TalonFXConfiguration config;

    private KrakenBuilder(int id, String can, String subsystem, String name) {
        motor = new Kraken(id, can, subsystem, name);
        config = new TalonFXConfiguration();
    }

    /**
     * Reduces CAN traffic for better performance
     * This is already integrated
     * @return itself
     */
    public KrakenBuilder optimizeBUSUtilization() {
        motor.optimizeBusUtilization();
        return this;
    }
    /**
     * Sets the inversion value of the motor
     * @param Invertedvalue is either InvertedValue.Clockwise_Positive or .CounterClockwise_Positive
     * The motor defaults to CounterClockwise_Positive
     * @return itself
     */
    public KrakenBuilder withInversion(InvertedValue Invertedvalue) {
        config.withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(Invertedvalue)
        );
        return this;
    }

    /**
     * Sets the idle state of the motor
     * @param neutralModeValue is either NeutralModeValue.Brake or .Coast
     * @return itself
     */
    public KrakenBuilder withIdleMode(NeutralModeValue neutralModeValue) {
        config.withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(neutralModeValue)
        );
        return this;
    }

    /**
     * Sets the current limits of the motor
     * @param limit is the limit for the motor
     * @return itself
     */
    public KrakenBuilder withCurrentLimit(double limit) {
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(limit)
                .withStatorCurrentLimitEnable(true)
        );
        return this;
    }

    /**
     * Makes the motor follow the leadMotor's current state and configuration
     * @param leadMotor is the lead motor itself
     * @param alignmentValue determines whether it is spinning the same or a different way. MotorAlighmentValue.Aligned or .Opposed
     * @return itself
     */
    public KrakenBuilder asFollower(TalonFX leadMotor, MotorAlignmentValue alignmentValue) {
        motor.setControl(new Follower(leadMotor.getDeviceID(), alignmentValue));
        return this;
    }

    /**
     * Makes the motor follow the leadMotor's current state and configuration, but ignores the leadMotors inversion settings
     * @param followerId is the leadMotor's id
     * @return itself
     */
    public KrakenBuilder asStrictFollower(int followerId) {
        motor.setControl(new StrictFollower(followerId));
        return this;
    }

    /**
     * Sets PID for the motor when Slot0 is selected
     * We currently don't have another Slot
     * @param p proportianal gain
     * @param i integral gain (rarely use, if ever)
     * @param d derivative gain
     * @return itslef
     */
     public KrakenBuilder withSlot0PID(double p, double i, double d) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        config.withSlot0(slot0Configs);
        return this;
    }

    /**
     * Sets PID for the motor when Slot0 is selected
     * We currently don't have another Slot
     * @param p proportianal gain
     * @param i integral gain (rarely use, if ever)
     * @param d derivative gain
     * @param s static feed forward
     * @return itslef
     */
    public KrakenBuilder withSlot0PIDS(double p, double i, double d, double s) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = s;
        config.withSlot0(slot0Configs);
        return this;
    }

    /**
     * Sets PID for the motor when Slot0 is selected
     * We currently don't have another Slot
     * @param p proportianal gain
     * @param i integral gain (don't use)
     * @param d derivative gain
     * @param g gravity feedforward / feedback gain
     * @return itself
     */
    public KrakenBuilder withSlot0PIDG(double p, double i, double d, double g) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kG = g;
        config.withSlot0(slot0Configs);
        return this;
    }

    /**
     * Sets PID for the motor when Slot0 is selected
     * We currently don't have another Slot
     * @param p proportianal gain
     * @param i integral gain (don't use)
     * @param d derivative gain
     * @param s static feed forward
     * @param g gravity feedforward / feedback gain
     * @return itself
     */
    public KrakenBuilder withSlot0PIDSG(double p, double i, double d, double s, double g) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = s;
        slot0Configs.kG = g;
        config.withSlot0(slot0Configs);
        return this;
    }

    /**
     * Makes a new KrakenBuilder
     * @param id is the id of the motor
     * @return a new KrakenBuilder
     */
    public static KrakenBuilder create(int id, String can, String subsystem, String name) {
        return new KrakenBuilder(id, can, subsystem, name).optimizeBUSUtilization();
    }

    /**
     * Makes a new KrakenBuilder with pre-defined defaults: current limit of 40 idle mode of Brake
     * @param id is the id of the motor
     * @return a new KrakenBUilder
     */
    public static KrakenBuilder createWithDefaults(int id, String can, String subsystem, String name) {
        return new KrakenBuilder(id, can, subsystem, name)
            .withCurrentLimit(40)
            .withIdleMode(NeutralModeValue.Brake)
            .optimizeBUSUtilization();
    }

    /**
     * Gets configuration and applies it to the motor
     * @return the motor
     */
    public Kraken build() {
        motor.getConfigurator().apply(config);
        return motor;
    } 

}
/*
* 
* @param p proportianal gain
* @param i integral gain (don't use)
* @param d derivative gain
* @param s static feed forward
* @param a acceleration feedforward gain
* @param v velocity feed forward gain
* @param g gravity feedforward / feedback gain
* @return itself
*/