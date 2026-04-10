package frc.robot.goals;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.VirtualSubsystem;

/**
 * Central robot state: what we're doing (goal).
 *
 * <p>Can be set by: - Teleop via RobotGoalsBehavior reacting to OperatorIntent - Autonomous via
 * direct command calls
 *
 * <p>TODO (students): Add goal triggers for each value in RobotGoal enum
 */
public class RobotGoals extends VirtualSubsystem implements RobotEvents {

  private static RobotGoals instance;

  private final EnumState<RobotGoal> currentGoal =
      new EnumState<>("RobotGoals/Goal", RobotGoal.IDLE);

  private RobotGoals() {}

  public Command setGoalCommand(RobotGoal goal) {
    return Commands.runOnce(() -> currentGoal.set(goal));
  }

  public static RobotGoals getInstance() {
    if (instance == null) {
      instance = new RobotGoals();
    }
    return instance;
  }

  // public RobotGoal getCurrentGoal() {
  //   return currentGoal.get();
  // }

  @Override
  public void periodic() {}

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(RobotGoal.IDLE);
  }

  public Trigger isAimingTrigger() {
    return currentGoal.is(RobotGoal.AIMING);
  }

  @Override
  public Trigger isIntakingTrigger() {
    return currentGoal.is(RobotGoal.INTAKING);
  }

  @Override
  public Trigger isOuttakingTrigger() {
    return currentGoal.is(RobotGoal.OUTTAKING);
  }

  @Override
  public Trigger isShootingTrigger() {
    return currentGoal.is(RobotGoal.SHOOTING);
  }

  @Override
  public Trigger isClimbingL0() {
    return currentGoal.is(RobotGoal.CLIMBINGL0);
  }

  @Override
  public Trigger isClimbingL1() {
    return currentGoal.is(RobotGoal.CLIMBINGL1);
  }

  @Override
  public Trigger isClimbingL2() {
    return currentGoal.is(RobotGoal.CLIMBINGL2);
  }

  @Override
  public Trigger isClimbingL3() {
    return currentGoal.is(RobotGoal.CLIMBINGL3);
  }

  @Override
  public Trigger isReverseIndexer() {
    return currentGoal.is(RobotGoal.REVERSE_INDEXER);
  }

  @Override
  public Trigger isRaisedIntakeTrigger() {
    return currentGoal.is(RobotGoal.RAISED_INTAKE);
  }

  @Override
  public Trigger isRaisingCompactorTrigger() {
    return currentGoal.is(RobotGoal.RAISING_COMPACTOR);
  }

  @Override
  public Trigger isLoweringCompactorTrigger() {
    return currentGoal.is(RobotGoal.LOWERING_COMPACTOR);
  }
}
