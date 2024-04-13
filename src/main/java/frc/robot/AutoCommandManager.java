package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
            
        PathPlannerAuto ampYAutoBlue = new PathPlannerAuto("AmpYBlueAuto");
        PathPlannerAuto ampYAutoRed = new PathPlannerAuto("AmpYRedAuto");
            
        PathPlannerAuto nonAmpJAutoBlue = new PathPlannerAuto("NonAmpJBlueAuto");
        PathPlannerAuto nonAmpJAutoRed = new PathPlannerAuto("NonAmpJRedAuto");
        
        PathPlannerAuto nonAmpYAutoBlue = new PathPlannerAuto("NonAmpYBlueAuto");
        PathPlannerAuto nonAmpYAutoRed = new PathPlannerAuto("NonAmpYRedAuto");

        PathPlannerAuto midUAuto = new PathPlannerAuto("MidUAuto");

        PathPlannerAuto ampLThreeAuto = new PathPlannerAuto("AmpLThreeAuto");
        PathPlannerAuto ampLTwoAuto = new PathPlannerAuto("AmpLTwoAuto");

        PathPlannerAuto ampTwoAuto = new PathPlannerAuto("AmpTwoAuto");
        PathPlannerAuto ampTwoAuto2 = new PathPlannerAuto("AmpTwoAuto");
        PathPlannerAuto midTwoAuto = new PathPlannerAuto("MidTwoAuto");
        PathPlannerAuto nonAmpTwoAuto = new PathPlannerAuto("NonAmpTwoAuto");

        PathPlannerAuto ampWaitAuto = new PathPlannerAuto("AmpWaitAuto");
        PathPlannerAuto midWaitAuto = new PathPlannerAuto("MidWaitAuto");
        PathPlannerAuto nonAmpWaitAuto = new PathPlannerAuto("NonAmpWaitAuto");

        // PathPlannerAuto nonAmpStage = new PathPlannerAuto("NonAmpStage");
        PathPlannerAuto nonAmpSkipYRed = new PathPlannerAuto("NonAmpSkipYRed");
        PathPlannerAuto ampSkipYRed = new PathPlannerAuto("AmpSkipYRed");


        m_chooser.setDefaultOption("None", null);

        m_chooser.addOption("MidWait", new WaitCommand(8.0).andThen(midTwoAuto));
        m_chooser.addOption("NonAmpWait", new WaitCommand(8.0).andThen(nonAmpTwoAuto));
        m_chooser.addOption("BLUE_AmpY", ampYAutoBlue);
        m_chooser.addOption("RED_AmpY", ampYAutoRed);
        m_chooser.addOption("BLUE_NonAmpJ", nonAmpJAutoBlue);
        m_chooser.addOption("RED_NonAmpJ", nonAmpJAutoRed);
        m_chooser.addOption("BLUE_NonAmpY", nonAmpYAutoBlue);
        m_chooser.addOption("RED_NonAmpY", nonAmpYAutoRed);
        m_chooser.addOption("MidUAuto", midUAuto);
        m_chooser.addOption("LTwo", ampLTwoAuto);
        m_chooser.addOption("LThree", ampLThreeAuto);
        m_chooser.addOption("AmpTwo", ampTwoAuto2);
        m_chooser.addOption("AmpWait", new WaitCommand(8.0).andThen(ampTwoAuto));
        m_chooser.addOption("MidTwo", midTwoAuto);
        m_chooser.addOption("NonAmpTwo", nonAmpTwoAuto);
        // m_chooser.addOption("NonAmpStage", nonAmpStage);
        m_chooser.addOption("NonAmpSkipYRed", nonAmpSkipYRed);
        m_chooser.addOption("AmpSkipYRed", ampSkipYRed);


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

        /*
         * - - Gamepiece Intake Commands - -
         * These commands attempt to intake a note at a given position. Assuming the game-piece camera can see the note, it will drive to it and intake it.
         * NOTE - These do NOT start the intake, they just tell the limelight to drive to a detected note.
         * 
         * Field layout:
         * (amp side)
         *  Alliance  MidLine
         *  |  top   |  Level 5
            |  mid   |  Level 4
         *  |  low   |  Level 3
         *  |        |  Level 2
         *  |        |  Level 1
         * (source side)
         */
        //TODO update all of the x and y positions for each of the alliance colors (don't do center line points)
        NamedCommands.registerCommand("AllianceTopNote", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(2.9, 7.0, new Rotation2d(0.0)), new Pose2d(13.68, 7.0, new Rotation2d(0.0))));
        NamedCommands.registerCommand("AllianceMidNote", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(2.9, 5.55, new Rotation2d(0.0)), new Pose2d(13.68, 5.55, new Rotation2d(0.0))));
        NamedCommands.registerCommand("AllianceLowNote", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(2.9, 4.1, new Rotation2d(0.0)), new Pose2d(13.68, 4.1, new Rotation2d(0.0))));
        NamedCommands.registerCommand("MidLineLevel1", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(8.29, 0.77, new Rotation2d(0.0)))
            .raceWith(indexer.newUntilNoteFoundCommand().withTimeout(1.0)));
        NamedCommands.registerCommand("MidLineLevel2", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(8.29, 2.44, new Rotation2d(0.0)))
            .raceWith(indexer.newUntilNoteFoundCommand().withTimeout(1.0)));
        NamedCommands.registerCommand("MidLineLevel3", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(8.29, 4.1, new Rotation2d(0.0)))
            .raceWith(indexer.newUntilNoteFoundCommand().withTimeout(1.0)));
        NamedCommands.registerCommand("MidLineLevel4", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(8.29, 5.78, new Rotation2d(0.0)))
            .raceWith(indexer.newUntilNoteFoundCommand().withTimeout(1.0)));
        NamedCommands.registerCommand("MidLineLevel5", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(8.29, 7.44, new Rotation2d(0.0)))
            .raceWith(indexer.newUntilNoteFoundCommand().withTimeout(1.0)));
        NamedCommands.registerCommand("MidLineLevel5.1", new LimeLightIntakeCommand(drivetrain, gamePieceUtility, 
            new Pose2d(8.29, 7.44, new Rotation2d(0.0))));

        // Waits until there is a note in the indexer or for 1 second. (So we don't move beyond the midline)
        NamedCommands.registerCommand("waitUntilNote", indexer.newUntilNoteFoundCommand().raceWith(new WaitCommand(1.0)));

        //Sets the shooter to a shooter preset
        NamedCommands.registerCommand("setFar", speakerUtil.setDesiredTargetCommand(Target.far));
        NamedCommands.registerCommand("setMid", speakerUtil.setDesiredTargetCommand(Target.medium));
        NamedCommands.registerCommand("setClose", speakerUtil.setDesiredTargetCommand(Target.close));

        //Prepares, shoots, and stops the shooter, pivot, and turret
        //NOTE - Can only be used when stationary
        NamedCommands.registerCommand("aimAndShoot",
            CommandFactoryUtility.createTurretPreaimCommand(turret)
                .andThen(CommandFactoryUtility.createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret, intake))
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret, intake))
        );

        //Shoots and stops the shooter, pivot, indexer, and turret
        NamedCommands.registerCommand("shoot",
            CommandFactoryUtility.createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret, intake)
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret, intake))
        );

        //Shoots the note currently in the shooter, does not stow.
        NamedCommands.registerCommand("shootNoStow", CommandFactoryUtility.createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret, intake));
        
        //Aims the turret instantaneously
        NamedCommands.registerCommand("aim", new TurretAimCommand(turret));

        //Turns on the intake without doing anything with the indexer
        NamedCommands.registerCommand("intakeNoIndexer", 
            CommandFactoryUtility.createIntakeNoIndexerCommand(intake));
        
        //Runs the intake and indexer to intake a note. Stops the intake once we get a note.
        NamedCommands.registerCommand("intake", 
            CommandFactoryUtility.createRunIntakeCommand(intake, indexer, turret));
        
        //Scores into the amp.
        NamedCommands.registerCommand("ampScore", 
            new PrintCommand("TODO add new amp command.") //TODO implement new amp here
        );  
        
        //Stops the intake and does a note backup
        NamedCommands.registerCommand("stopIntake", CommandFactoryUtility.createNoteBackUpCommand(indexer, intake, true));
        NamedCommands.registerCommand("stopIntakeSensor", CommandFactoryUtility.createNoteBackUpCommand(indexer, intake, true));

        //TODO: Ask harry what exactly it does. I know about as much about it as the name indicates.
        NamedCommands.registerCommand("movingSideShoot", 
            CommandFactoryUtility.createPrepareShootCommand(turret, pivot, shooter, 39.0)
                .andThen(CommandFactoryUtility.createShootPreparedCommand(indexer, intake))
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, turret))
                .andThen(pivot.newSetPosCommand(35.0)) // pivot angle for pillar shot
        );
        NamedCommands.registerCommand("movingSideShootPlus", 
            CommandFactoryUtility.createPrepareShootCommand(turret, pivot, shooter, 40.0)
                .andThen(CommandFactoryUtility.createShootPreparedCommand(indexer, intake))
                .andThen(CommandFactoryUtility.createStopShootingCommand(shooter, indexer, turret))
                .andThen(pivot.newSetPosCommand(35.0)) // pivot angle for pillar shot
        );
        
        // Turns the turret and pivot towards the speaker and spins up the shooter.
        NamedCommands.registerCommand("prepareShoot", CommandFactoryUtility.createPrepareShootCommand(turret, pivot, shooter, null));

        // The same as prepareShoot except we keep turning the pivot.
        NamedCommands.registerCommand("prepareShootEndless", CommandFactoryUtility.createPrepareShootEndlessCommand(turret, pivot, shooter, null));
        
        // Shoots assuming the shooter is already spun up and prepared to shoot. Does not stop the shooter.
        NamedCommands.registerCommand("preparedShoot", CommandFactoryUtility.createShootPreparedCommand(indexer, intake));

        // Stops the shooter and stows the indexer and pivot
        NamedCommands.registerCommand("stopShoot", CommandFactoryUtility.createStopShootingCommand(shooter, indexer, pivot, turret, intake));
        
        // Prepares to shoot from the shooting position in NonAmpY
        NamedCommands.registerCommand("prepareNonAmpYShoot", CommandFactoryUtility.createPreparePosedShootEndlessCommand(turret, pivot, shooter, 
            SpeakerScoreUtility.computePivotAnglePolyModel(SpeakerScoreUtility.inchesToSpeaker(
                true, 
                new Pose2d(convertBlueXToRedX(3.98), 2.0, new Rotation2d(0.0)),
                new Pose2d(3.98, 2.0, new Rotation2d(0.0))))
                - 0.0,    //degrees fudge factor
            new Pose2d(convertBlueXToRedX(3.98), 2.0, new Rotation2d(0.0)),
            new Pose2d(3.98, 2.0, new Rotation2d(0.0))));

        // Shoots from the shooting position used in AmpY
        // Used in AmpY3 & AmpY5
        NamedCommands.registerCommand("prepareAmpYShoot", CommandFactoryUtility.createPreparePosedShootEndlessCommand(turret, pivot, shooter,  
        SpeakerScoreUtility.computePivotAnglePolyModel(SpeakerScoreUtility.inchesToSpeaker(
            true, 
            new Pose2d(convertBlueXToRedX(5.38), 6.89, new Rotation2d(0.0)),
            new Pose2d(5.38, 6.89, new Rotation2d(0.0))))
            - 0.0,    //degrees fudge factor
        new Pose2d(convertBlueXToRedX(5.38), 6.89, new Rotation2d(0.0)),
        new Pose2d(5.38, 6.89, new Rotation2d(0.0))));

        NamedCommands.registerCommand("prepareAmpSkipYShoot3", CommandFactoryUtility.createPreparePosedShootEndlessCommand(turret, pivot, shooter,  
                SpeakerScoreUtility.computePivotAnglePolyModel(SpeakerScoreUtility.inchesToSpeaker(
                        true, 
                        new Pose2d(convertBlueXToRedX(5.39), 6.82, new Rotation2d(0.0)),
                        new Pose2d(5.39, 6.82, new Rotation2d(0.0))))
                        + 1.0,    //degrees fudge factor
                new Pose2d(convertBlueXToRedX(5.39), 7.3, new Rotation2d(0.0)),
                new Pose2d(5.39, 6.82, new Rotation2d(0.0))));

        NamedCommands.registerCommand("prepareAmpSkipYShoot4", CommandFactoryUtility.createPreparePosedShootEndlessCommand(turret, pivot, shooter, 
                SpeakerScoreUtility.computePivotAnglePolyModel(SpeakerScoreUtility.inchesToSpeaker(
                        true, 
                        new Pose2d(convertBlueXToRedX(5.39), 6.82, new Rotation2d(0.0)),
                        new Pose2d(5.39, 6.82, new Rotation2d(0.0))))
                        + 0.0,    //degrees fudge factor
                new Pose2d(convertBlueXToRedX(5.39), 7.32, new Rotation2d(0.0)),
                new Pose2d(5.39, 6.82, new Rotation2d(0.0))));

        NamedCommands.registerCommand("prepareAmpSkipYShoot5", CommandFactoryUtility.createPreparePosedShootEndlessCommand(turret, pivot, shooter, 
                SpeakerScoreUtility.computePivotAnglePolyModel(SpeakerScoreUtility.inchesToSpeaker(
                        true, 
                        new Pose2d(convertBlueXToRedX(5.39), 6.82, new Rotation2d(0.0)),
                        new Pose2d(5.39, 6.82, new Rotation2d(0.0))))
                        + 0.5,    //degrees fudge factor
                new Pose2d(convertBlueXToRedX(5.39), 7.32, new Rotation2d(0.0)),
                new Pose2d(5.39, 6.82, new Rotation2d(0.0))));

        NamedCommands.registerCommand("prepareSkipYShoot3", CommandFactoryUtility.createPreparePosedShootEndlessCommand(turret, pivot, shooter,  
                SpeakerScoreUtility.computePivotAnglePolyModel(SpeakerScoreUtility.inchesToSpeaker(
                        true, 
                        new Pose2d(convertBlueXToRedX(3.35), 3.0, new Rotation2d(0.0)),
                        new Pose2d(3.35, 3.0, new Rotation2d(0.0))))
                        + 0.0,    //degrees fudge factor
                new Pose2d(convertBlueXToRedX(3.35), 3.0, new Rotation2d(0.0)),
                new Pose2d(3.35, 3.0, new Rotation2d(0.0))));

        NamedCommands.registerCommand("prepareSkipYShoot4", CommandFactoryUtility.createPreparePosedShootEndlessCommand(turret, pivot, shooter, 
                SpeakerScoreUtility.computePivotAnglePolyModel(SpeakerScoreUtility.inchesToSpeaker(
                        true, 
                        new Pose2d(convertBlueXToRedX(3.35), 3.0, new Rotation2d(0.0)),
                        new Pose2d(3.35, 3.0, new Rotation2d(0.0))))
                        - 0.0,    //degrees fudge factor
                new Pose2d(convertBlueXToRedX(3.35), 3.0, new Rotation2d(0.0)),
                new Pose2d(3.35, 3.0, new Rotation2d(0.0))));



        NamedCommands.registerCommand("nonAmpSideShootNoStop", 
            CommandFactoryUtility.createPreparePosedShootCommand(turret, pivot, shooter, null,
                    new Pose2d(convertBlueXToRedX(1.43), 3.12, new Rotation2d(0.0)),
                    new Pose2d(1.43, 4.12, new Rotation2d(0.0)) )
                .andThen(CommandFactoryUtility.createShootPreparedCommand(indexer, intake)));
        
        // Offsets future shots down by one degree
        // THIS MUST BE RESET AFTER USE or it will mess up all future shots. In addition, this only affects the SpeakerScoreUtility calculated shot.
        // ! - - - !THIS MUST BE RESET AFTER USE! - - - !
        NamedCommands.registerCommand("setShotOffset-Down1",  new InstantCommand(()->SpeakerScoreUtility.setShotOffset(-1.0)));
        NamedCommands.registerCommand("setShotOffset-Down2",  new InstantCommand(()->SpeakerScoreUtility.setShotOffset(-2.5)));
        NamedCommands.registerCommand("setShotOffset-Up1",  new InstantCommand(()->SpeakerScoreUtility.setShotOffset(1.0)));
        NamedCommands.registerCommand("resetShotOffset",  new InstantCommand(()->SpeakerScoreUtility.resetShotOffset()));

        // Sets the shooter's target speed to 100 rots/s instead of 117. 
        // Used for slowing close-range shots so they don't bounce out of the speaker.
        // THIS MUST BE RESET AFTER USE or it will mess up all future shots. In addition, this only affects the SpeakerScoreUtility calculated shot.
        // ! - - - !THIS MUST BE RESET AFTER USE! - - - !
        NamedCommands.registerCommand("setShotSpeedOverride-100",  new InstantCommand(()->SpeakerScoreUtility.setShotSpeedOffset(100.0)));
        NamedCommands.registerCommand("resetShotSpeedOverride",  new InstantCommand(()->SpeakerScoreUtility.resetShotSpeedOffset()));

        NamedCommands.registerCommand("stopDrivetrain", drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withDriveRequestType(DriveRequestType.OpenLoopVoltage)));
    }

    private static double convertBlueXToRedX(double x) {
        return 16.5 - x;
    }
}