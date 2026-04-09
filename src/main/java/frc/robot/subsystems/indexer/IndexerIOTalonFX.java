package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {

  private VelocityVoltage indexerRequest;
  private TalonFX indexerMotor;
  private VelocityVoltage kickerRequest;
  private TalonFX kickerMotor;

  private AngularVelocity indexerSetPoint = RPM.of(0);
  private AngularVelocity kickerSetPoint = RPM.of(0);

  private static final double SENSOR_MECH_INDEXER = 1;
  private static final double KICKER_GEAR_RATIO = 3.0; // 3/1
  private static final double INDEXER_GEAR_RATIO = 30 / 18;

  private final NeutralOut m_neutralOut = new NeutralOut();

  public IndexerIOTalonFX(int indexerMotorCAN, CANBus canbus, int kickerMotorCAN) {
    indexerMotor = new TalonFX(indexerMotorCAN, canbus);
    kickerMotor = new TalonFX(kickerMotorCAN, canbus);
    indexerRequest = new VelocityVoltage(RPM.of(0.0)).withEnableFOC(false).withSlot(0);
    kickerRequest = new VelocityVoltage(RPM.of(0.0)).withEnableFOC(false).withSlot(0);

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration configIndexer = new TalonFXConfiguration();
    configIndexer.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configIndexer.CurrentLimits.StatorCurrentLimit = 80.0;
    configIndexer.CurrentLimits.StatorCurrentLimitEnable = true;
    configIndexer.CurrentLimits.SupplyCurrentLimit = 40.0;
    configIndexer.CurrentLimits.SupplyCurrentLimitEnable = true;
    configIndexer.Feedback.SensorToMechanismRatio = INDEXER_GEAR_RATIO;
    configIndexer.Voltage.PeakForwardVoltage = 12.0;
    configIndexer.Voltage.PeakReverseVoltage = -12.0;
    configIndexer.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> indexerMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(configIndexer));

    TalonFXConfiguration configKicker = new TalonFXConfiguration();
    configKicker.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configKicker.CurrentLimits.StatorCurrentLimit = 80.0;
    configKicker.CurrentLimits.StatorCurrentLimitEnable = true;
    configKicker.CurrentLimits.StatorCurrentLimit = 40.0;
    configKicker.CurrentLimits.StatorCurrentLimitEnable = true;
    configKicker.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;
    configKicker.Voltage.PeakForwardVoltage = 12.0;
    configKicker.Voltage.PeakReverseVoltage = -12.0;
    configKicker.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> kickerMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(configKicker));
  }

  @Override
  public void setIndexerTarget(AngularVelocity velocity) {
    if (velocity.in(RPM) != indexerSetPoint.in(RPM)) {
      indexerMotor.setControl(indexerRequest.withVelocity(velocity));
      indexerSetPoint = velocity;
    }
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
    inputs.indexerVelocity.mut_replace(indexerMotor.getVelocity().getValue());
    inputs.indexerSupplyCurrent.mut_replace(indexerMotor.getSupplyCurrent().getValue());
    inputs.indexerSetPoint.mut_replace(indexerSetPoint);
    inputs.indexerVoltage.mut_replace(indexerMotor.getMotorVoltage().getValue());
    inputs.indexerTorqueCurrent.mut_replace(indexerMotor.getTorqueCurrent().getValue());

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

    // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    // PhoenixUtil.tryUntilOk(5, () ->
    //   indexerMotor.getConfigurator().apply(motionMagicConfigs));
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

// UwU
