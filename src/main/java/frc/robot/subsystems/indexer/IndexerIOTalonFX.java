package frc.robot.subsystems.indexer;

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

public class IndexerIOTalonFX implements IndexerIO {
  private VelocityTorqueCurrentFOC indexerRequest;
  private TalonFX indexerMotor;
  private VelocityTorqueCurrentFOC kickerRequest;
  private TalonFX kickerMotor;

  private AngularVelocity indexerSetPoint = RPM.of(0);
  private AngularVelocity kickerSetPoint = RPM.of(0);

  private static final double KICKER_GEAR_RATIO = 24.0 / 18.0;
  private static final double INDEXER_GEAR_RATIO = 30.0 / 18.0;

  private final NeutralOut m_neutralOut = new NeutralOut();

  public IndexerIOTalonFX(int indexerMotorCAN, CANBus canbus, int kickerMotorCAN) {
    indexerMotor = new TalonFX(indexerMotorCAN, canbus);
    kickerMotor = new TalonFX(kickerMotorCAN, canbus);
    indexerRequest = new VelocityTorqueCurrentFOC(RPM.of(0.0)).withSlot(0);
    kickerRequest = new VelocityTorqueCurrentFOC(RPM.of(0.0)).withSlot(0);

    configureTalons();
  }

  private void configureTalons() {

    TalonFXConfiguration configKicker = new TalonFXConfiguration();
    configKicker.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configKicker.CurrentLimits.StatorCurrentLimit = 80.0;
    configKicker.CurrentLimits.StatorCurrentLimitEnable = true;
    configKicker.CurrentLimits.SupplyCurrentLimit = 30.0;
    configKicker.CurrentLimits.SupplyCurrentLimitEnable = true;
    configKicker.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    configKicker.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    configKicker.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;
    configKicker.Voltage.PeakForwardVoltage = 12.0;
    configKicker.Voltage.PeakReverseVoltage = -12.0;
    configKicker.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> kickerMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(configKicker));

    TalonFXConfiguration configIndexer = new TalonFXConfiguration();
    configIndexer.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configIndexer.CurrentLimits.StatorCurrentLimit = 80.0;
    configIndexer.CurrentLimits.StatorCurrentLimitEnable = true;
    configIndexer.CurrentLimits.SupplyCurrentLimit = 30.0;
    configIndexer.CurrentLimits.SupplyCurrentLimitEnable = true;
    configIndexer.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    configIndexer.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    configIndexer.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;
    configIndexer.Voltage.PeakForwardVoltage = 12.0;
    configIndexer.Voltage.PeakReverseVoltage = -12.0;
    configIndexer.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> indexerMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(configIndexer));
    indexerMotor.setControl(new Follower(kickerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void setKickerTarget(AngularVelocity velocity) {
    if (velocity.in(RPM) != kickerSetPoint.in(RPM)) {
      kickerMotor.setControl(kickerRequest.withVelocity(velocity));
      kickerSetPoint = velocity;
    }
  }

  @Override
  public void stop() {
    indexerMotor.setControl(m_neutralOut);
    indexerSetPoint = RPM.of(0.0);
    kickerMotor.setControl(m_neutralOut);
    kickerSetPoint = RPM.of(0.0);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.kickerVelocity.mut_replace(kickerMotor.getVelocity().getValue());
    inputs.kickerSupplyCurrent.mut_replace(kickerMotor.getSupplyCurrent().getValue());
    inputs.kickerSetPoint.mut_replace(kickerSetPoint);
    inputs.kickerVoltage.mut_replace(kickerMotor.getMotorVoltage().getValue());
    inputs.kickerTorqueCurrent.mut_replace(kickerMotor.getTorqueCurrent().getValue());
  }

  public void setIndexerGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(slot0Configs));
  }

  public void setKickerGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(slot0Configs));
  }
}
