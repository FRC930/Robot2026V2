package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {

  private Angle hoodAngle = Degrees.mutable(0);

  private final SingleJointedArmSim hoodSim;
  private ArmFeedforward ff = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(0.3, 0.0, 0.0, new Constraints(360.0, 720.0));

  // physical constants for hood (NOT ACCURATE)
  private static final DCMotor kArmMotor = DCMotor.getKrakenX60(1); // e.g., one NEO motor
  private static final double kGearing = 50.0; // e.g., 50:1 gear ratio
  private static final double kMoI = 1.5; // Moment of inertia in kg/m^2 (from CAD)
  private static final double kArmLength = Units.inchesToMeters(30.0); // e.g., 30 inches long
  private static final double kMinAngle = Units.degreesToRadians(-90.0); // e.g., -90 degrees
  private static final double kMaxAngle = Units.degreesToRadians(90.0); // e.g., 90 degrees
  private static final boolean kSimulateGravity = true;

  public HoodIOSim() {
    hoodSim =
        new SingleJointedArmSim(
            kArmMotor, kGearing, kMoI, kArmLength, kMinAngle, kMaxAngle, kSimulateGravity, 0);
  }

  @Override
  public void setHoodTarget(Angle angle) {
    this.hoodAngle = angle;
  }

  @Override
  public void stop() {
    this.hoodAngle = Radians.mutable(0);
  }

  public void updateInputs(HoodInputs inputs) {
    Voltage volts = updateHoodPID();

    inputs.hoodAngle.mut_replace(hoodSim.getAngleRads(), Radians);
    inputs.hoodSetAngle.mut_replace(hoodAngle);
    inputs.hoodVoltage.mut_replace(volts);
    inputs.hoodSupplyCurrent.mut_replace(hoodSim.getCurrentDrawAmps(), Amps);
  }

  private Voltage updateHoodPID() {
    Angle currentAngle = Radians.of(hoodSim.getAngleRads());

    Voltage controllerVoltage =
        Volts.of(controller.calculate(currentAngle.in(Degrees), hoodAngle.in(Degree)));
    Voltage feedForwardVoltage =
        Volts.of(
            ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    hoodSim.setInputVoltage(effort.in(Volts));
    hoodSim.update(0.02);
    return effort;
  }
}
