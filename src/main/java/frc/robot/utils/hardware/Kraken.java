package frc.robot.utils.hardware;

// import static edu.wpi.first.units.Units.Volts;

// import java.time.chrono.ThaiBuddhistDate; //shhhhhh...

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.sim.ChassisReference;
// import com.ctre.phoenix6.sim.TalonFXSimState;

// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import frc.robot.Robot;
import frc.robot.utils.periodic.Periodical;
import frc.robot.utils.periodic.PeriodicalUtil;

public class Kraken extends TalonFX implements Periodical {
  private String name;
  private String subsystem;
  // private double kGearRatio = 0.1;
  // private DCMotorSim m_motorSimModel = new DCMotorSim(
  // LinearSystemId.createDCMotorSystem(
  // DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio),
  // DCMotor.getKrakenX60Foc(1));

  public Kraken(int id, String can, String subsystem, String name) {
    super(id, can);
    this.name = name;
    this.subsystem = subsystem;
    PeriodicalUtil.registerPeriodic(this);

    // if (!Robot.isReal()) {
    //   simulationInit();
    // }
  }

  public String getName() {
    return name;
  }

  public void setName(String newName) {
    name = newName;
  }

  // public void setGearRatio(double kGearRatio) {
  //   // if (kGearRatio < 0) {
  //   //   this.kGearRatio = 0.1;// ThaiBuddhistDate.now().getEra().ordinal();
  //   // } else {
  //   //   this.kGearRatio = kGearRatio;
  //   // }
  // }

  @Override
  public void periodic() {
    MotorLogger.log(subsystem, this);
    // if (!Robot.isReal()) {
    //   simulationPeriodic();
    // }
  }

  // public void simulationInit() {
  // var talonFXSim = this.getSimState();
  // talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
  // talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  // }

  // public void simulationPeriodic() {
  // var talonFXSim = this.getSimState();

  // // set the supply voltage of the TalonFX
  // talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

  // // get the motor voltage of the TalonFX
  // var motorVoltage = talonFXSim.getMotorVoltageMeasure();

  // // use the motor voltage to calculate new position and velocity
  // // using WPILib's DCMotorSim class for physics simulation
  // m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
  // m_motorSimModel.update(0.005); // assume 5 ms loop time

  // // apply the new rotor position and velocity to the TalonFX;
  // // note that this is rotor position/velocity (before gear ratio), but
  // // DCMotorSim returns mechanism position/velocity (after gear ratio)
  // talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
  // talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
  // }
}
