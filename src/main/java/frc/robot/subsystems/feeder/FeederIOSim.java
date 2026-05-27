package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {

  private AngularVelocity feederAppliedVelocity = RPM.mutable(0.0);
  private final FlywheelSim feederSim;
  private SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(0.0, 0.002, 0.0);
  private PIDController feederPID = new PIDController(0.0031, 0.0, 0.0);

  public FeederIOSim() {
    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
  }

  @Override
  public void setFeederTarget(AngularVelocity velocity) {
    this.feederAppliedVelocity = velocity;
  }

  @Override
  public void stop() {
    this.feederAppliedVelocity = RPM.zero();
  }

  @Override
  public void updateInputs(FeederInputs inputs) {

    inputs.feederVelocity.mut_replace(feederSim.getAngularVelocity());
    inputs.feederSetPoint.mut_replace(feederAppliedVelocity);
    inputs.feederVoltage.mut_replace(Volts.of(feederSim.getInputVoltage()));

    updateFeederPID();
  }

  private void updateFeederPID() {
    // Current velocity from simulation
    double currentVelocity = feederSim.getAngularVelocity().in(RPM);
    double targetVelocity = feederAppliedVelocity.in(RPM);

    // PID output (in volts) based on velocity error
    double pidOutput = feederPID.calculate(currentVelocity, targetVelocity);

    // Feedforward voltage for target velocity
    double ffOutput = feederFF.calculate(targetVelocity);

    // Total voltage command
    double totalVoltage = pidOutput + ffOutput;

    // Clamp voltage to [-12V, 12V] to simulate real battery limits
    totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));
    // Apply voltage to simulation
    feederSim.setInputVoltage(totalVoltage);

    // Advance simulation
    feederSim.update(.02);
  }
}
