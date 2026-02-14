package frc.robot.utils.hardware;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.utils.periodic.Periodical;
import frc.robot.utils.periodic.PeriodicalUtil;

public class Kraken extends TalonFX implements Periodical{
    private String name;
    private String subsystem;

    public Kraken(int id, String can, String subsystem, String name) {
        super(id, can);
        this.name = name;
        this.subsystem = subsystem;
        PeriodicalUtil.registerPeriodic(this);
    }

    public String getName() {
        return name;
    }

    public void setName(String newName) {
        name = newName;
    }

    @Override
    public void periodic() {
        MotorLogger.log(subsystem, this);
    }
}
