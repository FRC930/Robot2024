package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LimeLightIntakeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.SetPivotPositionCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.TurretAimCommand;
import frc.robot.commands.TurretRefineCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.LimeLightDetectionUtility;
import frc.robot.utilities.SpeakerScoreUtility;
import frc.robot.utilities.SpeakerScoreUtility.Target;

public class AutoCommandManager {

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public AutoCommandManager(SwerveDrivetrainSubsystem drivetrain, 
        LimeLightDetectionUtility gamePieceUtility,
        TurretSubsystem turret, 
        ShooterSubsystem shooter, 
        IndexerSubsystem indexer, 
        SpeakerScoreUtility speakerUtil, 
        IntakeSubsystem intake, 
        PivotSubsystem pivot) {
        configureNamedCommands(drivetrain, 
            gamePieceUtility, 
            turret, 
            shooter, 
            indexer,  
            speakerUtil, 
            intake, 
            pivot);
        //m_chooser = AutoBuilder.buildAutoChooser();
        PathPlannerAuto rotationTest = new PathPlannerAuto("RotationTest");
        PathPlannerAuto centerShootWCommand = new PathPlannerAuto("CenterShoot(x2)(W)");
        PathPlannerAuto centerShootYCommand = new PathPlannerAuto("CenterShoot(x1)(Y)");
        // PathPlannerAuto wingShoot3Command = new PathPlannerAuto("WingShoot(x3)(L)");
        PathPlannerAuto wingShoot2Command = new PathPlannerAuto("WingShoot(x2)(L)");
        PathPlannerAuto topWingShootCommand = new PathPlannerAuto("TopWingShoot(x1)");
        PathPlannerAuto midWingShootCommand = new PathPlannerAuto("MidWingShoot(x1)");
        PathPlannerAuto botWingShootCommand = new PathPlannerAuto("BotWingShoot(x1)");
        PathPlannerAuto NonAmpYCommand = new PathPlannerAuto("NonAmpYAuto");
        PathPlannerAuto NonAmpYNoStowCommand = new PathPlannerAuto("NonAmpYWOStow");

        m_chooser.setDefaultOption("None", null);
        m_chooser.addOption("RotationTest", rotationTest);
        m_chooser.addOption("WCenterShoot", centerShootWCommand);
        m_chooser.addOption("YCenterShoot", centerShootYCommand);
        m_chooser.addOption("NonAmpYAuto", NonAmpYCommand);
        m_chooser.addOption("NonAmpYNoStow", NonAmpYNoStowCommand);
        m_chooser.addOption("LWingShoot", wingShoot2Command);
        // m_chooser.addOption("LWingShoot", wingShoot3Command);
        m_chooser.addOption("TopWingShoot(x1)", topWingShootCommand);
        m_chooser.addOption("MidWingShoot(x1)", midWingShootCommand);
        m_chooser.addOption("BotWingShoot(x1)", botWingShootCommand);

        SmartDashboard.putData("SelectAuto", m_chooser);
    }

    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }

    public Command getAutoManagerSelected(){
        return m_chooser.getSelected();
    }

    public void configureNamedCommands(SwerveDrivetrainSubsystem drivetrain, 
        LimeLightDetectionUtility gamePieceUtility, 
        TurretSubsystem turret, 
        ShooterSubsystem shooter, 
        IndexerSubsystem indexer, 
        SpeakerScoreUtility speakerUtil, 
        IntakeSubsystem intake, 
        PivotSubsystem pivot) { 
        //TODO update all of the x and y positions for each of the alliance colors (don't do center line points)
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

        NamedCommands.registerCommand("setFar", speakerUtil.setDesiredTargetCommand(Target.far));
        NamedCommands.registerCommand("setMid", speakerUtil.setDesiredTargetCommand(Target.medium));
        NamedCommands.registerCommand("setClose", speakerUtil.setDesiredTargetCommand(Target.close));
        NamedCommands.registerCommand("aimAndShoot",
            CommandFactoryUtility.createTurretPreaimCommand(turret)
                .andThen(CommandFactoryUtility.createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret))
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret))
        );
        NamedCommands.registerCommand("shoot",
            CommandFactoryUtility.createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret)
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret))
        );
        NamedCommands.registerCommand("shootNoStow", CommandFactoryUtility.createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret));
        NamedCommands.registerCommand("aim", new TurretAimCommand(turret));
        NamedCommands.registerCommand("closeAim", new TurretAimCommand(turret, true));
        NamedCommands.registerCommand("intakeNoIndexer", 
            CommandFactoryUtility.createIntakeNoIndexerCommand(intake));
        NamedCommands.registerCommand("intake", 
            CommandFactoryUtility.createRunIntakeCommand(intake, indexer, turret));
        NamedCommands.registerCommand("ampScore", 
            CommandFactoryUtility.createAmpScoreCommand(pivot, turret, shooter, indexer)
                .andThen(CommandFactoryUtility.createNoteBackUpCommand(indexer, intake))
        );    
        NamedCommands.registerCommand("stopIntake", CommandFactoryUtility.createNoteBackUpCommand(indexer, intake));
        NamedCommands.registerCommand("sideWingScore", 
            CommandFactoryUtility.createTurretPreaimCommand(turret, true)
                .andThen(CommandFactoryUtility.createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret, 45.0))
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret))
        );
        NamedCommands.registerCommand("closeAimAndShoot", 
            CommandFactoryUtility.createTurretPreaimCommand(turret, true)
                .andThen(CommandFactoryUtility.createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret))
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret))
        );
        NamedCommands.registerCommand("noAmpYScore1", 
            CommandFactoryUtility.createPrepareShootCommand(turret, pivot, shooter, 45.0, true)
                .andThen(CommandFactoryUtility.createShootPreaimedCommand(shooter, pivot, indexer, turret))
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, turret))
                .andThen(pivot.newSetPosCommand(35.0)) // pivot angle for pillar shot
        );
        NamedCommands.registerCommand("noAmpYScore2",
            CommandFactoryUtility.createPrepareShootCommand(turret, shooter, true)
                .andThen(CommandFactoryUtility.createShootPreaimedCommand(shooter, pivot, indexer, turret))
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret))
        );
        NamedCommands.registerCommand("fullAim", CommandFactoryUtility.createPrepareShootCommand(turret, pivot, shooter, false));
        NamedCommands.registerCommand("preAimedShoot", CommandFactoryUtility.createShootPreaimedCommand(shooter, pivot, indexer, turret));
        NamedCommands.registerCommand("stopShoot", CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret));

    }
}