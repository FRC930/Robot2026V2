package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.aiming.AimingService;
import frc.robot.commands.DriveCommands;
import frc.robot.goals.RobotGoal;
import frc.robot.goals.RobotGoals;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Optional;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private Drive m_drive;
  private final String autoWaitKey = "AutoWait";

  public static double autoWaitTime = 0.0;

  public AutoCommandManager(Drive drive, RobotGoals goals, AimingService aimingService) {
    SmartDashboard.putNumber(autoWaitKey, autoWaitTime);
    configureNamedCommands(drive, goals, aimingService);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    for (String autoName : AutoBuilder.getAllAutoNames()) {
      try {
        // Validate if path .0 exists
        PathPlannerPath path = PathPlannerPath.fromPathFile(autoName + ".0");
        File f =
            new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto");
        JSONObject autoJson =
            (JSONObject) new JSONParser().parse(new FileReader((f.getAbsoluteFile())));
        boolean resetOdom =
            autoJson.get("resetOdom") != null && (boolean) autoJson.get("resetOdom");
        if (!resetOdom) {
          autoChooser.addOption(autoName, new AutoPathCommand(autoName));
        }
      } catch (FileNotFoundException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  public Command getAutonomousCommand() {
    return getAutoWithCurrentPose();
  }

  public Command elasticWaitCommand() {
    return new WaitCommand(SmartDashboard.getNumber("AutoWait", 0.0));
  }

  private void configureNamedCommands(Drive drive, RobotGoals goals, AimingService AimingService) {
    NamedCommands.registerCommand("Intaking", goals.setGoalCommand(RobotGoal.INTAKING));
    NamedCommands.registerCommand(
        "Shooting",
        goals
            .setGoalCommand(RobotGoal.SHOOTING)
            .alongWith(DriveCommands.autoAimForAutoNoExit(drive, AimingService).withTimeout(2.0)));
    NamedCommands.registerCommand("Outtaking", goals.setGoalCommand(RobotGoal.OUTTAKING));
    // TODO: Make only the intake retract
    NamedCommands.registerCommand("Idle", goals.setGoalCommand(RobotGoal.IDLE));
    NamedCommands.registerCommand("WaitTime", elasticWaitCommand());
    NamedCommands.registerCommand("SyncOdometry", DriveCommands.syncOdometry(drive));
    // NamedCommands.registerCommand(
    // "AutoAim", DriveCommands.joystickDrive(drive, AimingService).withTimeout(1.0));
  }

  public Command getAutoWithCurrentPose() {
    Command command = autoChooser.get();
    Command returnCommand = command;

    if (command instanceof AutoPathCommand) {
      AutoPathCommand ppAutoCommand = (AutoPathCommand) command;
      String autoName = ppAutoCommand.m_autoName;
      // TODO detemine if need to prepend (super class with attribute of auto name)
      returnCommand = getToPath(autoName + ".0");
      if (returnCommand != null) {
        returnCommand = returnCommand.andThen(command);
      }
    }
    return returnCommand;
  }

  public Command getToPath(String pathName) {
    Command command = null;
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Optional<Pose2d> optPath = path.getStartingHolonomicPose();
      if (optPath.isPresent()) {
        // NOTE: Make sure .0 path and starting velocity is > 0.0
        Pose2d pose;
        if (AutoBuilder.shouldFlip()) {
          pose = FlippingUtil.flipFieldPose(optPath.get());
        } else {
          pose = optPath.get();
        }
        command =
            AutoBuilder.pathfindToPose(
                pose, path.getGlobalConstraints(), path.getIdealStartingState().velocity());
      }
    } catch (Exception e) {
      // If we get Exception assume we have no command
      command = null;
    }
    return command;
  }
}
