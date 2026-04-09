package frc.robot.subsystems.feeder;

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

public class FeederIOTalonFX implements FeederIO {

  private VelocityVoltage feederRequest;
  private TalonFX feederMotor;

  private AngularVelocity feederSetPoint = RPM.of(0);

  private static final double SENSOR_MECH_INDEXER = 24;

  private final NeutralOut m_neutralOut = new NeutralOut();

  public FeederIOTalonFX(int feederMotorCAN, CANBus canbus) {
    feederMotor = new TalonFX(feederMotorCAN, canbus);
    feederRequest = new VelocityVoltage(RPM.of(0.0)).withEnableFOC(false).withSlot(0);
    configureTalons();
  }

  private void configureTalons() {

    TalonFXConfiguration configFeeder = new TalonFXConfiguration();
    configFeeder.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configFeeder.CurrentLimits.StatorCurrentLimit = 80.0;
    configFeeder.CurrentLimits.StatorCurrentLimitEnable = true;
    configFeeder.CurrentLimits.SupplyCurrentLimit = 40.0;
    configFeeder.CurrentLimits.SupplyCurrentLimitEnable = true;
    configFeeder.Voltage.PeakForwardVoltage = 12.0;
    configFeeder.Voltage.PeakReverseVoltage = -12.0;
    configFeeder.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> feederMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(configFeeder));
  }

  @Override
  public void setFeederTarget(AngularVelocity velocity) {
    if (velocity.in(RPM) != feederSetPoint.in(RPM)) {
      feederMotor.setControl(feederRequest.withVelocity(velocity));
      feederSetPoint = velocity;
    }
  }

  @Override
  public void stop() {
    feederMotor.setControl(m_neutralOut);
    feederSetPoint = RPM.of(0.0);
  }

  @Override
  public void updateInputs(FeederInputs inputs) {
    inputs.feederVelocity.mut_replace(feederMotor.getVelocity().getValue());
    inputs.feederSupplyCurrent.mut_replace(feederMotor.getSupplyCurrent().getValue());
    inputs.feederSetPoint.mut_replace(feederSetPoint);
    inputs.feederVoltage.mut_replace(feederMotor.getMotorVoltage().getValue());
    inputs.feederTorqueCurrent.mut_replace(feederMotor.getTorqueCurrent().getValue());
  }

  // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
  // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
  // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
  // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
  // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
  // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
  // PhoenixUtil.tryUntilOk(5, () ->
  //   indexerMotor.getConfigurator().apply(motionMagicConfigs));

  public void setFeederGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(slot0Configs));

    // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    // PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
