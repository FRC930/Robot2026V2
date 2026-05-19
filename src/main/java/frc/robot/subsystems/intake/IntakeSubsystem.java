// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.aiming.AimingService;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Sets the controless the intake and endexer */
public class IntakeSubsystem extends SubsystemBase implements IntakeEvents {
  /** Creates a new ExampleSubsystem. */
  private IntakeIO m_IO;

  private LoggedTunableNumber intakeTargetRPM =
      new LoggedTunableNumber("Intake/intakeTargetRPM", 4000.0);
  private final EnumState<IntakeState> currentGoal =
      new EnumState<>("Intake/States", IntakeState.IDLE);

  public LoggedTunableGainsBuilder rollerGains =
      new LoggedTunableGainsBuilder(
          "Gains/IntakeSubsystem/", 15.0, 0, 0.1, 3.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  private IntakeInputsAutoLogged logged = new IntakeInputsAutoLogged();

  public IntakeSubsystem(IntakeIO IO) {
    m_IO = IO;
    logged.rollerVelocity = RPM.mutable(0.0);
    logged.rollerVelocitySetPoint = RPM.mutable(0.0);
    logged.rollerSupplyCurrent = Amps.mutable(0.0);
    logged.rollerTorqueCurrent = Amps.mutable(0);
    logged.rollerVoltage = Volts.mutable(0);
    logged.leaderRollerTemp = Fahrenheit.mutable(0);
    logged.followerRollerTemp = Fahrenheit.mutable(0);
    m_IO.setRollerGains(rollerGains.build());
    logged.numberFuelHave = 0;
  }

  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public void setIntakeSpeed(AngularVelocity speed) {
    m_IO.setRollerTargetSpeed(speed);
  }

  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public Command intakeCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.INTAKING);
        });
  }

  public Command outtakeCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.OUTTAKING);
        });
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.IDLE);
        });
  }

  public Command shootingCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.SHOOTING);
        });
  }

  public Command raisedCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.RAISED);
        });
  }

  public Command agitateCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.AGITATING);
        });
  }

  public void setTestingState() {
    currentGoal.set(IntakeState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Intake", logged);
    switch (currentGoal.get()) {
      case SHOOTING:
        if (Constants.currentMode == Constants.Mode.SIM) {
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.shootFuel();
            sim.setRunning(false);
          }
        }
        break;
      case INTAKING:
        m_IO.setRollerTargetSpeed(RPM.of(intakeTargetRPM.get()));
        if (Constants.currentMode == Constants.Mode.SIM) {
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.setRunning(true);
          }
        }
        break;
      case OUTTAKING:
        m_IO.setRollerTargetSpeed(RPM.of(-intakeTargetRPM.get()));
        if (Constants.currentMode == Constants.Mode.SIM) {
          AimingService.trajectorySim.setSpawnFuelOnGround(false);
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.setRunning(false);
          }
        }
        break;
      case RAISED:
        m_IO.setRollerTargetSpeed(RPM.of(intakeTargetRPM.get()));
        if (Constants.currentMode == Constants.Mode.SIM) {
          AimingService.trajectorySim.setSpawnFuelOnGround(false);
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.setRunning(false);
          }
        }
        break;
      case IDLE:
        stop();
        if (Constants.currentMode == Constants.Mode.SIM) {
          AimingService.trajectorySim.setSpawnFuelOnGround(false);
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.setRunning(false);
          }
        }
        break;
      case AGITATING:
        m_IO.stop();
        if (Constants.currentMode == Constants.Mode.SIM) {
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.shootFuel();
            sim.setRunning(false);
          }
        }
        break;
    }
    rollerGains.ifGainsHaveChanged((gains) -> this.m_IO.setRollerGains(gains));
  }

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(IntakeState.IDLE);
  }

  @Override
  public Trigger isIntakingTrigger() {
    return currentGoal.is(IntakeState.INTAKING);
  }

  @Override
  public Trigger isOuttakingTrigger() {
    return currentGoal.is(IntakeState.OUTTAKING);
  }

  @Override
  public Trigger isShootingTrigger() {
    return currentGoal.is(IntakeState.SHOOTING);
  }

  @Override
  public Trigger isRaisedTrigger() {
    return currentGoal.is(IntakeState.RAISED);
  }

  public Command getNewSetIntakeVelocityCommand(DoubleSupplier rpm) {
    return new InstantCommand(
        () -> {
          m_IO.setRollerTargetSpeed(RPM.of(rpm.getAsDouble()));
        },
        this);
  }
}
