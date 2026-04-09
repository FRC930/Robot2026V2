package frc.robot.subsystems.extender;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ExtenderIOTalonFX implements ExtenderIO {
  public MotionMagicTorqueCurrentFOC Request;
  public TalonFX extenderMotor;
  public TalonFX followerMotor;

  public ExtenderInputs inputs;

  private Distance m_setPoint = Distance.ofBaseUnits(0, Inches);

  private final NeutralOut m_brake = new NeutralOut();

  public ExtenderIOTalonFX(int extenderMotorID, int followerMotorID, CANBus canbus) {
    extenderMotor = new TalonFX(extenderMotorID, canbus);
    followerMotor = new TalonFX(followerMotorID, canbus);
    m_setPoint = Inches.of(0.0);
    Request = new MotionMagicTorqueCurrentFOC(0);

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfgExtender = new TalonFXConfiguration();
    cfgExtender.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfgExtender.Voltage.PeakForwardVoltage = 12;
    cfgExtender.Voltage.PeakReverseVoltage = 12;
    cfgExtender.CurrentLimits.StatorCurrentLimit = 80;
    cfgExtender.CurrentLimits.StatorCurrentLimitEnable = true;
    cfgExtender.CurrentLimits.SupplyCurrentLimit = 30;
    cfgExtender.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfgExtender.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfgExtender.Feedback.SensorToMechanismRatio = ExtenderSubsystem.REDUCTION;

    PhoenixUtil.tryUntilOk(5, () -> extenderMotor.getConfigurator().apply(cfgExtender));

    TalonFXConfiguration cfgFollower = new TalonFXConfiguration();
    cfgFollower.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfgFollower.Voltage.PeakForwardVoltage = 12;
    cfgFollower.Voltage.PeakReverseVoltage = 12;
    cfgFollower.CurrentLimits.StatorCurrentLimit = 80;
    cfgFollower.CurrentLimits.StatorCurrentLimitEnable = true;
    cfgFollower.CurrentLimits.SupplyCurrentLimit = 30;
    cfgFollower.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfgFollower.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfgFollower.Feedback.SensorToMechanismRatio = ExtenderSubsystem.REDUCTION;

    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(cfgFollower));
    followerMotor.setControl(
        new Follower(extenderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(ExtenderInputs inputs) {
    double rotations = extenderMotor.getPosition().getValue().in(Rotations);
    inputs.distance.mut_replace(Inches.of(rotations * ExtenderSubsystem.INCHES_PER_ROT));
    inputs.velocity.mut_replace(
        InchesPerSecond.of(extenderMotor.getVelocity().getValue().in(RotationsPerSecond)));
    inputs.setPoint.mut_replace(m_setPoint);
    inputs.supplyCurrent.mut_replace(extenderMotor.getSupplyCurrent().getValue());
    inputs.voltage.mut_replace(extenderMotor.getMotorVoltage().getValue());
  }

  @Override
  public void setExtenderHeight(Distance target) {
    Request =
        Request.withPosition(target.in(Inches) / ExtenderSubsystem.INCHES_PER_ROT).withSlot(0);
    extenderMotor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void stop() {
    extenderMotor.setControl(m_brake);
  }

  @Override
  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    PhoenixUtil.tryUntilOk(5, () -> extenderMotor.getConfigurator().apply(slot0Configs));

    // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    // PhoenixUtil.tryUntilOk(5, () -> extenderMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
