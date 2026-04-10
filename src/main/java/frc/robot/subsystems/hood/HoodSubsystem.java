package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotVisualization;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase implements HoodEvents {
  // Implementation goes here​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​​ † ​₀
  // ​​​​​​​​​​​ᴥ ₀​​ †

  private LoggedTunableNumber aimAngle = new LoggedTunableNumber("Hood/aimAngle", 45);
  private LoggedTunableNumber passAngle = new LoggedTunableNumber("Hood/passAngle", 22.5);

  private final HoodIO m_IO;
  private volatile boolean shouldThreadCommand = false;

  private final EnumState<HoodState> currentGoal = new EnumState<>("Hood/States", HoodState.IDLE);

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/HoodSubsystem/", 450.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private HoodInputsAutoLogged logged = new HoodInputsAutoLogged();

  private final DoubleSupplier hoodAngleSupplier;

  public HoodSubsystem(HoodIO IO, DoubleSupplier hoodAngleSupplier) {
    m_IO = IO;
    this.hoodAngleSupplier = hoodAngleSupplier;
    logged.hoodAngle = Degrees.mutable(0);
    logged.hoodSetAngle = Degrees.mutable(0);
    logged.hoodVoltage = Volts.mutable(0);
    logged.hoodSupplyCurrent = Amps.mutable(0);
    logged.hoodTorqueCurrent = Amps.mutable(0);

    m_IO.setGains(tunableGains.build());
    RobotVisualization.instance().setHoodAngleSource(logged.hoodAngle);
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          currentGoal.set(HoodState.IDLE);
        });
  }

  public Command aimCommand() {
    return runOnce(
        () -> {
          currentGoal.set(HoodState.AIMING);
        });
  }

  public void setTestingState() {
    currentGoal.set(HoodState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Hood", logged);
    HoodState state = currentGoal.get();
    shouldThreadCommand = (state == HoodState.AIMING);
    switch (state) {
      case IDLE:
        m_IO.setHoodTarget(Degrees.of(0.0));
        break;
      case AIMING:
        break; // ​250Hz ​​thread​ handles​​ ​​​​motor​ commands
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
  }

  public boolean shouldThreadCommand() {
    return shouldThreadCommand;
  }

  public HoodIO getIO() {
    return m_IO;
  }

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(HoodState.IDLE);
  }

  @Override
  public Trigger isAimingTrigger() {
    return currentGoal.is(HoodState.AIMING);
  }

  public Command getNewSetHoodAngleCommand(DoubleSupplier angle) {
    return new InstantCommand(
        () -> {
          m_IO.setHoodTarget(Degrees.of(angle.getAsDouble()));
        },
        this);
  }
}
