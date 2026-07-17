package frc.robot.subsystems.feeder;

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

public class FeederIOTalonFX implements FeederIO {

  private VelocityTorqueCurrentFOC feederRequest;
  private TalonFX feederMotor;

  private TalonFX feederMotorFollower;

  private AngularVelocity feederSetPoint = RPM.of(0);

  private final NeutralOut m_neutralOut = new NeutralOut();

  public FeederIOTalonFX(int feederMotorCAN, int feederMotorFollowerCAN, CANBus canbus) {
    feederMotor = new TalonFX(feederMotorCAN, canbus);
    feederMotorFollower = new TalonFX(feederMotorFollowerCAN, canbus);
    feederRequest = new VelocityTorqueCurrentFOC(RPM.of(0.0));

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration configFeeder = new TalonFXConfiguration();
    configFeeder.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configFeeder.CurrentLimits.StatorCurrentLimit = 90.0;
    configFeeder.CurrentLimits.StatorCurrentLimitEnable = true;
    configFeeder.CurrentLimits.SupplyCurrentLimit = 35.0;
    configFeeder.CurrentLimits.SupplyCurrentLimitEnable = true;
    configFeeder.Voltage.PeakForwardVoltage = 12.0;
    configFeeder.Voltage.PeakReverseVoltage = -12.0;
    configFeeder.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> feederMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(configFeeder));

    TalonFXConfiguration configFollower = new TalonFXConfiguration();
    configFollower.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configFollower.CurrentLimits.StatorCurrentLimit = 90.0;
    configFollower.CurrentLimits.StatorCurrentLimitEnable = true;
    configFollower.CurrentLimits.SupplyCurrentLimit = 35.0;
    configFollower.CurrentLimits.SupplyCurrentLimitEnable = true;
    configFollower.Voltage.PeakForwardVoltage = 12.0;
    configFollower.Voltage.PeakReverseVoltage = -12.0;
    configFollower.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> feederMotorFollower.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> feederMotorFollower.getConfigurator().apply(configFollower));
    feederMotorFollower.setControl(
        new Follower(feederMotor.getDeviceID(), MotorAlignmentValue.Aligned));
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

  public void setFeederGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(slot0Configs));
  }
}
