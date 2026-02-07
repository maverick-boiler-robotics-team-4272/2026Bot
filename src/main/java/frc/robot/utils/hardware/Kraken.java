package frc.robot.utils.hardware;

import com.ctre.phoenix6.hardware.TalonFX;

public class Kraken extends TalonFX{
    private String name;

    public Kraken(int id, String can, String name) {
        super(id, can);
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void setName(String newName) {
        name = newName;
    }
}
