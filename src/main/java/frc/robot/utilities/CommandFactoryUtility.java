package frc.robot.utilities;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TurretAimCommand;
import frc.robot.subsystems.AmpHoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LeafBlower.BlowerSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public final class CommandFactoryUtility {
    private static final double PIVOT_FEED_POS = 45.0;

    //#region positions
    public static final double TURRET_STOW_POS = 0.0;               /*Deg*/

    public static final double ELEVATOR_STOW_POS = 0.0;             /*Deg*/
    public static final double ELEVATOR_AMP_POS = 0.0;              /*Deg*/
    
    public static final double PIVOT_STOW_POS = 0.0;                /*Deg*/
    public static final double PIVOT_INTAKE_POS = 45.0;             /*Deg*/

    public static final double INDEXER_SPEAKER_SPEED = 0.9;         /*Value*/
    public static final double INDEXER_INTAKE_SPEED = 0.5;          /*Value*/
    public static final double INDEXER_REVERSE_SPEED = -0.1;        /*value*/
    public static final double INDEXER_FEED_SPEED = 0.9;            /*Value*/

    private static final double SHOOTER_LEFT_FEED_SPEED = 74.0;     /*Rot/s*/
    private static final double SHOOTER_RIGHT_FEED_SPEED = 74.0;    /*Rot/s*/
    
    public static final double LEFT_SHOOTER_EJECT_SPEED = 40.0;     /*Rot/s*/
    public static final double RIGHT_SHOOTER_EJECT_SPEED = 40.0;    /*Rot/s*/
    public static final double INDEXER_EJECT_SPEED = -0.2;          /*Value*/

    public static final double INTAKE_SPEED = 0.90;                 /*Value*/
    public static final double INTAKE_SHOOTING_SPEED = 0.90;        /*Value*/
    public static final double INTAKE_REJECT_SPEED = 0.0;//-0.15;         /*Value*/
    private static final double INTAKE_EJECT_SPEED = -0.4;          /*value*/

    public static final double ELEVATOR_CLIMB_POS = 8.0;

    public static final double PIVOT_TIMEOUT = 1.0;                 /*sec*/
    public static final double ELEVATOR_TIMEOUT = 1.0;              /*sec*/
    public static final double TURRET_TIMEOUT = 1.0;                /*sec*/
    private static final double SHOOTER_TIMEOUT = 1.0;              /*sec*/
    private static final double AFTER_SHOOT_TIMEOUT = 0.2;          /*sec*/
    private static final double AFTER_AMP_SHOOT_TIMEOUT = 0.4;      /*sec*/

    private static final double TURRET_PREAIM_TIMEOUT = 0.75;       /*sec*/

    private static final double STAR_AMP_VEL = 60.0; // 65.0; // 70.0;
    public static final double LEFT_SHOOTER_AMP_SPEED = 40.0;       /*Rot/s*/
    public static final double RIGHT_SHOOTER_AMP_SPEED = 40.0;      /*Rot/s*/
    private static final double AMP_STAR_PIVOT_POS = 20.0;          // 20.0;
    private static final double INDEXER_AMP_SPEED = -5.5; //-6.0

    public static Command createEjectCommand(TurretSubsystem turret, IndexerSubsystem indexer, IntakeSubsystem intake) {
            return turret.newSetPosCommand(TURRET_STOW_POS)
                .andThen(turret.newWaitUntilSetpointCommand(TURRET_TIMEOUT))
                .andThen(indexer.newSetSpeedCommand(INDEXER_EJECT_SPEED))
                .andThen(intake.newSetSpeedCommand(INTAKE_EJECT_SPEED))
                .andThen(indexer.newUntilNoNoteFoundCommand()) // dont stop until note gone
                .andThen(intake.newSetSpeedCommand(INTAKE_SHOOTING_SPEED))
                .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    public static Command createStopShootingCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, PivotSubsystem pivot,  TurretSubsystem turret, IntakeSubsystem intake) {
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .alongWith(indexer.newSetSpeedCommand(0.0))
            .alongWith(intake.newSetSpeedCommand(INTAKE_REJECT_SPEED))
            .alongWith(pivot.newSetPosCommand(PIVOT_STOW_POS))
            .alongWith(turret.newSetPosCommand(TURRET_STOW_POS));
    }

    public static Command createHardStopShootingCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, PivotSubsystem pivot,  TurretSubsystem turret, IntakeSubsystem intake) {
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .alongWith(indexer.newSetSpeedCommand(0.0))
            .alongWith(intake.newSetSpeedCommand(INTAKE_REJECT_SPEED))
            .alongWith(pivot.newSetPosCommand(PIVOT_STOW_POS))
            .alongWith(turret.newSetPosCommand(TURRET_STOW_POS))
            .andThen(new WaitCommand(0.5).andThen(shooter.newSetSpeedsWithSlotCommand(-1.0, -1.0, 1)));
    }

    /**
     * This doesn't stow pivot, use createStopShootingCommand(ShooterSubsystem, IndexerSubsystem, PivotSubsystem, TurretSubsystem) if pivot should stow
     */
    public static Command createStopShootingCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret) {
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .alongWith(indexer.newSetSpeedCommand(0.0))
            .alongWith(turret.newSetPosCommand(TURRET_STOW_POS));
    }
    
    public static Command createStopIntakingCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return intake.newSetSpeedCommand(0.0)
            .andThen(indexer.newSetSpeedCommand(0.0));
            // .andThen(new InstantCommand(() -> 
            //     {LimelightHelpers.setLEDMode_ForceOff("limelight-front"); 
            //     LimelightHelpers.setLEDMode_ForceOff("limelight-back");}));
    }

    public static Command createNoteBackUpCommand(IndexerSubsystem indexer, IntakeSubsystem intake, boolean isAuto) {
        return new ConditionalCommand(
            indexer.newSetSpeedCommand(INDEXER_REVERSE_SPEED)
            .andThen(new WaitCommand(isAuto ? 0.40 : 0.20))
            .andThen(indexer.newSetSpeedCommand(0.0)),
            new InstantCommand(),
            () -> (isAuto || indexer.getSensorDistance() >= 40))
                .andThen(CommandFactoryUtility.createStopIntakingCommand(intake, indexer))
                    .andThen(intake.newSetSpeedCommand(CommandFactoryUtility.INTAKE_REJECT_SPEED));
    }

    public static Command createRunIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, TurretSubsystem turret) {
        return indexer.newUntilNoNoteFoundCommand()  // make sure no note is found
            .andThen(turret.newSetPosCommand(TURRET_STOW_POS))
            .andThen(turret.newWaitUntilSetpointCommand(0.3))
            .andThen(intake.newSetSpeedCommand(INTAKE_SPEED)
                .alongWith(indexer.newSetSpeedsCommand(0.3, 0.3)))
            .andThen(indexer.newUntilNoteFoundCommand())
            .andThen(new WaitCommand(0.05))
            // .alongWith(new InstantCommand(() -> 
            //     {LimelightHelpers.setLEDMode_ForceOn("limelight-front"); 
            //     LimelightHelpers.setLEDMode_ForceOn("limelight-back");})) // Wait on the intake, we're stopping too quickly
            // // .andThen(createStopIntakingCommand(intake, indexer)) // currently used separately, only add if told
            .andThen(indexer.newSetSpeedCommand(0.0))
            .andThen(intake.newSetSpeedCommand(0.0)); // Dont stop intake until note found
    }

    // public static Command createElevatorClimbCommand(ElevatorSubsystem elevator) {
    //     return elevator.newSetPosCommand(ELEVATOR_CLIMB_POS)
    //         .andThen(elevator.newWaitUntilSetpointCommand(ELEVATOR_TIMEOUT));
    // }

    // public static Command createStowElevatorCommand(ElevatorSubsystem elevator) {
    //     return elevator.newSetPosCommand(ELEVATOR_STOW_POS);
    // }
    public static Command createPivotAndShooterSpeedCommand(ShooterSubsystem shooter, PivotSubsystem pivot, Double pivotAngle) {
        Command command = shooter.newCalcAndSetSpeedsCommand(); //shooter.newSetSpeedsCommand(speakerUtil)
            if (pivotAngle == null) {
                command = command.andThen(pivot.newCalcAndSetPosCommand()); //.andThen(pivot.newSetPosCommand(speakerUtil))
            } else {
                command = command.andThen(pivot.newSetPosCommand(pivotAngle));
            }
        return command;
    }

    public static Command createIntakeNoIndexerCommand(IntakeSubsystem intake){
        return intake.newSetSpeedCommand(INTAKE_SPEED);
    }
    
    public static Command createSpeakerScoreCommand(SpeakerScoreUtility speakerUtil, ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, TurretSubsystem turret, IntakeSubsystem intake, Double pivotAngle, boolean adjustPivot) {
        Command command = null;
        if (adjustPivot){
            command = createPivotAndShooterSpeedCommand(shooter, pivot, pivotAngle);
        } else {
            command = new InstantCommand();
        }

        return command 
            .andThen(createShootPreaimedCommand(shooter, pivot, indexer, turret, intake));
    }

    public static Command createShootPreaimedCommand(ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, TurretSubsystem turret, IntakeSubsystem intake) {
        return (pivot.newWaitUntilSetpointCommand(0.7)
            .alongWith(shooter.newWaitUntilSetpointCommand(0.7))
            .alongWith(turret.newWaitUntilSetpointCommand(0.7)))
        .andThen(indexer.newSetSpeedCommand(INDEXER_SPEAKER_SPEED))
        .andThen(intake.newSetSpeedCommand(INTAKE_SHOOTING_SPEED))
        .andThen(indexer.newUntilNoNoteFoundCommand()) // dont stop until note gone
        .andThen(createLogShotCommand("Shoot Preaimed"))
        .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    public static Command createShootPreparedCommand(IndexerSubsystem indexer, IntakeSubsystem intake) {
        return indexer.newSetSpeedCommand(INDEXER_SPEAKER_SPEED)
        .andThen(intake.newSetSpeedCommand(INTAKE_SHOOTING_SPEED))
        .andThen(indexer.newUntilNoNoteFoundCommand()) // dont stop until note gone
        .andThen(createLogShotCommand("Shoot Prepared"))
        .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }
        
    public static Command createSpeakerScoreCommand(SpeakerScoreUtility speakerUtil, ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, TurretSubsystem turret, IntakeSubsystem intake, Double pivotAngle) {
    {
        return createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret, intake, pivotAngle, true);
    }}

    public static Command createSpeakerScoreCommand(SpeakerScoreUtility speakerUtil, ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, TurretSubsystem turret, IntakeSubsystem intake) {
        return createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret, intake, null);
    }

    public static Command createTurretPreaimCommand(TurretSubsystem turret) {
        return new TurretAimCommand(turret)
           .raceWith(turret.newWaitUntilSetpointCommand(TURRET_PREAIM_TIMEOUT));
    }

    public static Command createPrepareShootCommand(TurretSubsystem turret, PivotSubsystem pivot, ShooterSubsystem shooter) {
        return createPrepareShootCommand(turret, pivot, shooter, null);
    }

    /**
     * @deprecated Do not use, ends as soon as pivot reaches target. 
     * use {@code createPreparePosedShootEndless()} with proxy poses instead.
     * <p> We moved over to a proxy-pose based method of shooting during autos because our odometry was quacked.
     * <>
     */
    @Deprecated
    public static Command createPrepareShootEndlessCommand(TurretSubsystem turret, PivotSubsystem pivot, ShooterSubsystem shooter, Double pivotAngle) {
        return new TurretAimCommand(turret)
            .alongWith(
                new RepeatCommand(
                    createPivotAndShooterSpeedCommand(shooter, pivot, pivotAngle)
                )
            )
        ;
    }

    /**
     * Prepares the turret, pivot, and shooter to shoot from a given pose and with a given pivot angle.
     * @param turret
     * @param pivot
     * @param shooter
     * @param pivotAngle
     * @param poseRed
     * @return
     */
    public static Command createPreparePosedShootEndlessCommand(TurretSubsystem turret, PivotSubsystem pivot, ShooterSubsystem shooter, Double pivotAngle, Pose2d poseRed, Pose2d poseBlue) {
        return new TurretAimCommand(turret, poseRed, poseBlue)
            .alongWith(
                new RepeatCommand(
                    createPivotAndShooterSpeedCommand(shooter, pivot, pivotAngle)
                )
            )
        ;
    }


    public static Command createPrepareShootCommand(TurretSubsystem turret, PivotSubsystem pivot, ShooterSubsystem shooter, Double pivotAngle) {
        return new TurretAimCommand(turret)
            .raceWith(turret.newWaitUntilSetpointCommand(TURRET_PREAIM_TIMEOUT))
            .alongWith(createPivotAndShooterSpeedCommand(shooter, pivot, pivotAngle))
            .andThen(pivot.newWaitUntilSetpointCommand(0.75)
                .alongWith(shooter.newWaitUntilSetpointCommand(0.75))
                .alongWith(turret.newWaitUntilSetpointCommand(0.75)));
    }


    public static Command createPreparePosedShootCommand(TurretSubsystem turret, PivotSubsystem pivot, ShooterSubsystem shooter, Double pivotAngle, Pose2d poseRed, Pose2d poseBlue) {
        return new TurretAimCommand(turret, poseRed, poseBlue)
            .raceWith(turret.newWaitUntilSetpointCommand(TURRET_PREAIM_TIMEOUT))
            .alongWith(createPivotAndShooterSpeedCommand(shooter, pivot, pivotAngle))
            .andThen(pivot.newWaitUntilSetpointCommand(0.55)
                .alongWith(shooter.newWaitUntilSetpointCommand(0.55))
                .alongWith(turret.newWaitUntilSetpointCommand(0.55)));
    }

    public static Command createPrepareShootCommand(TurretSubsystem turret, ShooterSubsystem shooter) {
        return new TurretAimCommand(turret)
            .raceWith(turret.newWaitUntilSetpointCommand(TURRET_PREAIM_TIMEOUT))
            .alongWith(shooter.newCalcAndSetSpeedsCommand())
            .andThen(shooter.newWaitUntilSetpointCommand(SHOOTER_TIMEOUT)
                .alongWith(turret.newWaitUntilSetpointCommand(TURRET_TIMEOUT)));
    }


    public static Command createFeedCommand(PivotSubsystem pivot, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        return pivot.newSetPosCommand(PIVOT_FEED_POS)
        .andThen(shooter.newSetSpeedsWithSlotCommand(SHOOTER_LEFT_FEED_SPEED,SHOOTER_RIGHT_FEED_SPEED,1))
        .andThen(pivot.newWaitUntilSetpointCommand(PIVOT_TIMEOUT)
                .alongWith(shooter.newWaitUntilSetpointCommand(SHOOTER_TIMEOUT))
                )
        .andThen(indexer.newSetSpeedCommand(INDEXER_FEED_SPEED))
        .andThen(intake.newSetSpeedCommand(INTAKE_SHOOTING_SPEED))
        .andThen(indexer.newUntilNoNoteFoundCommand()) // dont stop until note gone
        .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    public static Command createPrepareStarAmpCommand(IndexerSubsystem indexer, TurretSubsystem turret,
            PivotSubsystem pivot) {
        return// indexer.newSetStarVoltageCommand(0.62)
        turret.newSetPosCommand(TURRET_STOW_POS)
        .andThen(pivot.newSetPosCommand(AMP_STAR_PIVOT_POS))
        .andThen(new InstantCommand(() -> turret.enableTurretLock(),turret))
        .andThen(indexer.newSetTopVoltageCommand(3.0)) // TODOspin 
        .andThen(indexer.newSetStarVoltageCommand(3.0))
        .andThen(new WaitCommand(0.2))
        .andThen(indexer.newSetTopVoltageCommand(0.0))
        .andThen(indexer.newSetStarVoltageCommand(0.0));
    }

    public static Command createAmpCommand(IndexerSubsystem indexer,TurretSubsystem turret, PivotSubsystem pivot) {
        return createPrepareStarAmpCommand(indexer, turret, pivot) 
        .alongWith(turret.newWaitUntilSetpointCommand(TURRET_TIMEOUT))
        .alongWith(pivot.newWaitUntilSetpointCommand(AFTER_AMP_SHOOT_TIMEOUT))
        .andThen(indexer.newSetStarMMVelocityCommand(STAR_AMP_VEL)); // Start shooting 
    }

    public static Command createStopAmpCommand(IndexerSubsystem indexer, TurretSubsystem turret, PivotSubsystem pivot, IntakeSubsystem intake) {
        return indexer.newSetTopVoltageCommand(INDEXER_AMP_SPEED) 
        .andThen(intake.newSetSpeedCommand(INTAKE_SHOOTING_SPEED))
        .andThen(indexer.newUntilNoNoteFoundCommand())
        .andThen(new WaitCommand(AFTER_AMP_SHOOT_TIMEOUT))
        .andThen(pivot.newSetPosCommand(0.0)
            .alongWith(indexer.newSetSpeedCommand(0.0))
            .alongWith(intake.newSetSpeedCommand(0.0))
        .andThen(new InstantCommand(() -> turret.disableTurretLock(),turret)));
    }

    public static Command createTestBlowerCommand(BlowerSubsystem blower){
        return blower.newSetSpeedCommand(1.0)
        .andThen(new WaitCommand(5.0))
        .andThen(blower.newSetSpeedCommand(0.0));
    }

    public static Command createLogShotCommand(String comment) {
        return ShotLoggingUtil.getAdvanceShotCommand()
        .andThen(ShotLoggingUtil.getPivotInstance().getDoLogCommand(comment))
        .andThen(ShotLoggingUtil.getTurretInstance().getDoLogCommand(comment));
    }
}
