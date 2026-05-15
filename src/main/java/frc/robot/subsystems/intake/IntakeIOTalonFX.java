package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  TalonFX followIntakeMotor;
  TalonFX leaderIntakeMotor;

  private VelocityTorqueCurrentFOC intakeRequest;
  private AngularVelocity intakeSetPoint = RPM.of(0);
  boolean firstTime = true;
  public static double GEAR_RATIO_ROLLERS = 4.0 / 3.0;

  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  public IntakeIOTalonFX(int IntakeLeadMotorCAN, int IntakeFollowMotorCAN, CANBus canbus) {
    leaderIntakeMotor = new TalonFX(IntakeLeadMotorCAN, canbus);
    followIntakeMotor = new TalonFX(IntakeFollowMotorCAN, canbus);
    intakeRequest = new VelocityTorqueCurrentFOC(RPM.of(0.0)).withSlot(0);
    configureTalons();
  }

  public void configureTalons() {
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    leaderConfig.Voltage.PeakForwardVoltage = 12.0;
    leaderConfig.Voltage.PeakReverseVoltage = -12.0;
    leaderConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO_ROLLERS;
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> leaderIntakeMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> leaderIntakeMotor.getConfigurator().apply(leaderConfig));

    // Follower runs lighter limits than the leader — both motors drive the
    // same shaft opposed, so total throughput is set by the leader. Dropping
    // the follower saves ~10-15 A of supply draw during intake spikes.
    TalonFXConfiguration followConfig = new TalonFXConfiguration();
    followConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    followConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    followConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    followConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    followConfig.Voltage.PeakForwardVoltage = 12.0;
    followConfig.Voltage.PeakReverseVoltage = -12.0;
    followConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO_ROLLERS;
    followConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followConfig.Feedback.RotorToSensorRatio = 1.0;
    PhoenixUtil.tryUntilOk(
        5, () -> followIntakeMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> followIntakeMotor.getConfigurator().apply(followConfig));

    followIntakeMotor.setControl(
        new Follower(leaderIntakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.rollerVelocity.mut_replace(leaderIntakeMotor.getVelocity().getValue());
    inputs.rollerVelocitySetPoint.mut_replace(intakeSetPoint);
    inputs.rollerSupplyCurrent.mut_replace(leaderIntakeMotor.getSupplyCurrent().getValue());
    inputs.rollerTorqueCurrent.mut_replace(leaderIntakeMotor.getTorqueCurrent().getValue());
    inputs.rollerVoltage.mut_replace(leaderIntakeMotor.getMotorVoltage().getValue());
    inputs.leaderRollerTemp.mut_replace(leaderIntakeMotor.getDeviceTemp().getValue());
    inputs.followerRollerTemp.mut_replace(followIntakeMotor.getDeviceTemp().getValue());
  }

  @Override
  public void stop() {
    leaderIntakeMotor.setControl(m_brake);
    intakeSetPoint = RPM.of(0.0);
  }

  @Override
  public void setRollerTargetSpeed(AngularVelocity target) {
    if (target.in(RPM) != intakeSetPoint.in(RPM)) {
      leaderIntakeMotor.setControl(intakeRequest.withVelocity(target));
      intakeSetPoint = target;
    }
  }

  @Override
  public void setRollerGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> leaderIntakeMotor.getConfigurator().apply(slot0Configs));
  }
}
