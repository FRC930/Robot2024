package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LimeLightIntakeCommand;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.utilities.GamePieceDetectionUtility;

public class AutoCommandManager {
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    public AutoCommandManager(SwerveDrivetrainSubsystem drivetrain, GamePieceDetectionUtility gamePieceUtility) {
        configureNamedCommands(drivetrain, gamePieceUtility);
        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("SelectAuto", m_chooser);
    }
    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }
    public Command getAutoManagerSelected(){
        return m_chooser.getSelected();
    }
    public void configureNamedCommands(SwerveDrivetrainSubsystem drivetrain, GamePieceDetectionUtility gamePieceUtility) { //TODO update all of the x and y positions for each of the alliance colors (don't do center line points)
      NamedCommands.registerCommand("AllianceTopNote", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
          new Pose2d(2.9, 7.0, new Rotation2d(0.0)), new Pose2d(13.68, 7.0, new Rotation2d(0.0))));
      NamedCommands.registerCommand("AllianceMidNote", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
          new Pose2d(2.9, 5.55, new Rotation2d(0.0)), new Pose2d(13.68, 5.55, new Rotation2d(0.0))));
      NamedCommands.registerCommand("AllianceLowNote", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
          new Pose2d(2.9, 4.1, new Rotation2d(0.0)), new Pose2d(13.68, 4.1, new Rotation2d(0.0))));
      NamedCommands.registerCommand("MidLineLevel1", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
          new Pose2d(8.29, 0.77, new Rotation2d(0.0))));
      NamedCommands.registerCommand("MidLineLevel2", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
          new Pose2d(8.29, 2.44, new Rotation2d(0.0))));
      NamedCommands.registerCommand("MidLineLevel3", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
          new Pose2d(8.29, 4.1, new Rotation2d(0.0))));
      NamedCommands.registerCommand("MidLineLevel4", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
          new Pose2d(8.29, 5.78, new Rotation2d(0.0))));
      NamedCommands.registerCommand("MidLineLevel5", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
          new Pose2d(8.29, 7.44, new Rotation2d(0.0))));
    }
}
