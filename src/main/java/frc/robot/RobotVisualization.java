package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class RobotVisualization extends VirtualSubsystem {
  private static RobotVisualization instance;

  private MutDistance extenderExtension = Inches.mutable(0.0);
  private MutAngle hoodAngle = Degrees.mutable(0.0);

  private final Mechanism2d primaryMechanism2d;

  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d baseLigament2d =
      new MechanismLigament2d("RobotBase", 150, 0, 24, new Color8Bit(Color.kBlue));

  @Override
  public void periodic() {
    visualize();
  }

  private final String key;

  private RobotVisualization(String key) {
    super();
    this.key = key;
    primaryMechanism2d = new Mechanism2d(500, 300);

    robotBaseRoot = primaryMechanism2d.getRoot("2dBaseRoot", 225, 20);
    robotBaseRoot.append(baseLigament2d);
  }

  public Distance getExtenderExtension() {
    return extenderExtension;
  }

  public void setExtenderExtension(Distance extenderExtension) {
    this.extenderExtension.mut_replace(extenderExtension);
  }

  public void setExtenderExtensionSource(MutDistance extenderExtension) {
    this.extenderExtension = extenderExtension;
  }

  public Angle getHoodAngle() {
    return hoodAngle;
  }

  public void setHoodAngle(Angle hoodAngle) {
    this.hoodAngle.mut_replace(hoodAngle);
  }

  public void setHoodAngleSource(MutAngle hoodAngle) {
    this.hoodAngle = hoodAngle;
  }

  public static RobotVisualization instance() {
    if (instance == null) {
      instance = new RobotVisualization("measured");
    }
    return instance;
  }

  private void visualize() {
    Pose3d extenderPose =
        new Pose3d(
            new Translation3d(this.getExtenderExtension(), Inches.zero(), Inches.zero()),
            Rotation3d.kZero);

    Pose3d hoodPose =
        new Pose3d(
            new Translation3d(
                Meters.of(-0.2667),
                Meters.zero(),
                Meters.of(0.4807)), // constant value for attachment offset
            new Rotation3d(Degrees.zero(), this.getHoodAngle().unaryMinus(), Degrees.zero()));

    Logger.recordOutput("RobotState/Extender/" + key, extenderPose);
    Logger.recordOutput("RobotState/Hood/" + key, hoodPose);
  }
}
