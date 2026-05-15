package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.Gains;

public class ShooterIOSim implements ShooterIO {
  private AngularVelocity shooterAngularVelocity = RPM.mutable(0.0);

  private final FlywheelSim shooterSim;

  public ShooterIOSim() {
    shooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
  }

  @Override
  public void setShooterTarget(AngularVelocity target) {
    this.shooterAngularVelocity = target;
  }

  @Override
  public void stop() {
    setShooterTarget(RPM.of(0));
  }

  @Override
  public void updateInputs(ShooterInputs input) {
    input.shooterAngularVelocity.mut_replace(shooterSim.getAngularVelocity());
    input.shooterSetpoint.mut_replace(shooterAngularVelocity);

    // Periodic
    shooterSim.setInputVoltage(shooterAngularVelocity.in(RPM));
    shooterSim.update(0.02);
  }

  @Override
  public void setGains(Gains gains) {}
}
