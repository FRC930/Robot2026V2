package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {

  public PositionVoltage request;

  public TalonFX motor;

  private Angle m_setAngle;

  public static final double MINANGLE = 2.0;
  public static final double MAXANGLE = 31.0;

  public HoodIOTalonFX(int motorID, CANBus canbus) {
    motor = new TalonFX(motorID, canbus);
    m_setAngle = Degrees.of(0.0);
    request = new PositionVoltage(Degrees.of(0.0)).withSlot(0);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.SupplyCurrentLimit = 20.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -60.0;
    cfg.Voltage.PeakForwardVoltage = 12.0;
    cfg.Voltage.PeakReverseVoltage = -3.0;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    cfg.Feedback.SensorToMechanismRatio =
        (32.0 / 11.0)
            * (156.00000
                / 10.0); // Combination of a 3:1 Ratio from the Motor Pinion to and a 15.4:1 Ratio
    // Pinion to
    // Hood
    cfg.Feedback.RotorToSensorRatio = 1.0;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(cfg));

    // Set the initial position of the extender to be up (so that our starting configuration is
    // within frame parameter + motors are intialized to correct positions)
    PhoenixUtil.tryUntilOk(5, () -> motor.setPosition(Degrees.of(MINANGLE)));
  }

  @Override
  public void setHoodTarget(Angle angle) {
    if (angle.in(Degrees) != m_setAngle.in(Degrees)) {
      angle = Degrees.of(MathUtil.clamp(angle.in(Degrees), MINANGLE, MAXANGLE));
      motor.setControl(request.withPosition(angle));
      m_setAngle = angle;
    }
  }

  @Override
  public void stop() {
    Angle pos = Degrees.of(2.0);
    request = request.withPosition(pos);
    motor.setControl(request);
    m_setAngle = pos;
  }

  @Override
  public void updateInputs(HoodInputs input) {
    input.hoodAngle.mut_replace(motor.getPosition().getValue());
    input.hoodSetAngle.mut_replace(m_setAngle);
    input.hoodVoltage.mut_replace(motor.getMotorVoltage().getValue());
    input.hoodSupplyCurrent.mut_replace(motor.getSupplyCurrent().getValue());
    input.hoodTorqueCurrent.mut_replace(motor.getTorqueCurrent().getValue());
  }

  @Override
  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(slot0Configs));
  }
}
