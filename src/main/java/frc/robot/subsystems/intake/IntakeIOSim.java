package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.aiming.AimingService;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSim;

  // physical constants for intake extender (NOT ACCURATE)
  private static final double kArmGearRatio = 1.0;
  private static final double kArmLengthMeters = Units.inchesToMeters(12.0);
  private static final double kArmMassKg = Units.lbsToKilograms(3.0);

  // intake extender stow (up) angle setpoint, in radians (0 is down, positive is up)

  // intake Roller
  private AngularVelocity m_rollerVelocitySetPoint = RPM.mutable(0.0);

  private final FlywheelSim rollerFlyWheelSim;
  private final BooleanSupplier isSolutionValid;
  private SimpleMotorFeedforward rollerFF = new SimpleMotorFeedforward(0.0, 0.002, 0.0);
  // NOTE: ProfilePID sorta worked if did not have any FF KV BUT did not reach goal
  // private ProfiledPIDController rollerPID =
  //     new ProfiledPIDController(0.0069, 0.0, 0.0, new Constraints(6000, 10000));
  private PIDController rollerPID = new PIDController(0.0031, 0.0, 0.0);

  private int counter = 0;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain, BooleanSupplier isSolutionValid) {

    // Setup Intake roller
    rollerFlyWheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);

    // TODO: FINAL INTAKE SPACE CONFIGURATION FOR MAPLE-SIM
    // Here, create the intake simulation with respect to the intake on your real robot
    this.intakeSim =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrain,
            // Width of the intake
            Meters.of(0.6),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 67 gamepiece
            100);

    this.isSolutionValid = isSolutionValid;
  }

  public void setRunning(boolean runIntake) {
    if (runIntake)
      intakeSim.startIntake(); // Extends the intake out from the chassis frame and starts detecting
    // contacts with game pieces
    else intakeSim.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece
    // collection
  }

  public Command setRunnningCommand(boolean runIntake) {
    return new InstantCommand(
        () -> {
          setRunning(runIntake);
        });
  }

  @Override
  public void setRollerTargetSpeed(AngularVelocity rpm) {
    m_rollerVelocitySetPoint = rpm;
  }

  @Override
  public void stop() {
    // If REAL robot use coast
    setRollerTargetSpeed(RPM.of(0));
    // Doing nothing with extender motor
    // setExtenderTargetAngle(Degrees.of(0));
  }

  public int getGamePiecesAmount() {
    return intakeSim.getGamePiecesAmount();
  }

  public static void spawnFuel(double x, double y) {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                new Translation2d(Inches.of(16), Inches.of(1)),
                new Translation2d(x, y),
                new ChassisSpeeds(0.1, 0.1, 0.1),
                new Rotation2d(Degrees.of(0.0)),
                Meters.of(0.1),
                MetersPerSecond.of(0),
                Degrees.of(0)));
  }

  @Override
  public void updateInputs(IntakeInputs input) {
    // update inputs

    //  - intake
    updateRollerPID();
    input.rollerVelocity.mut_replace(rollerFlyWheelSim.getAngularVelocity());
    input.rollerVelocitySetPoint.mut_replace(m_rollerVelocitySetPoint);
    input.rollerSupplyCurrent.mut_replace(rollerFlyWheelSim.getCurrentDrawAmps(), Amps);

    input.numberFuelHave = getGamePiecesAmount();
  }

  private void updateRollerPID() {
    // Current velocity from simulation
    double currentVelocity = rollerFlyWheelSim.getAngularVelocity().in(RPM);
    double targetVelocity = m_rollerVelocitySetPoint.in(RPM);

    // PID output (in volts) based on velocity error
    double pidOutput = rollerPID.calculate(currentVelocity, targetVelocity);

    // Feedforward voltage for target velocity
    double ffOutput = rollerFF.calculate(targetVelocity);
    // Logger.recordOutput("TOTALFF", ffOutput);

    // Total voltage command
    double totalVoltage = pidOutput + ffOutput;

    // Logger.recordOutput("TOTALVOLTS", totalVoltage);
    // Clamp voltage to [-12V, 12V] to simulate real battery limits
    totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));
    // Logger.recordOutput("TOTALVOLTSCLAMP", totalVoltage);
    // Apply voltage to simulation
    rollerFlyWheelSim.setInputVoltage(totalVoltage);

    // Advance simulation
    rollerFlyWheelSim.update(.02);
  }

  public void shootFuel() {
    counter++;
    if (counter > 3.5) {
      if (intakeSim.getGamePiecesAmount() > 0) {
        if (isSolutionValid.getAsBoolean()) {
          intakeSim.obtainGamePieceFromIntake();
          intakeSim.obtainGamePieceFromIntake();
          intakeSim.obtainGamePieceFromIntake();
          AimingService.trajectorySim.setSpawnFuelOnGround(true);
        } else {
          AimingService.trajectorySim.setSpawnFuelOnGround(false);
        }
      } else {
        AimingService.trajectorySim.setSpawnFuelOnGround(false);
      }
      counter = 0;
    } else {
      AimingService.trajectorySim.setSpawnFuelOnGround(false);
    }
  }
}
