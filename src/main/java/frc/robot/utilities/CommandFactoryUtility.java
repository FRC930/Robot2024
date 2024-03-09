package frc.robot.utilities;

import java.util.function.ObjDoubleConsumer;

import org.littletonrobotics.conduit.schema.CoreInputs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HoodCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.TurretAimCommand;
import frc.robot.commands.TurretRefineCommand;
import frc.robot.subsystems.AmpHoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public final class CommandFactoryUtility {

    //#region positions
    public static final double TURRET_STOW_POS = 0.0;               /*Deg*/

    public static final double ELEVATOR_STOW_POS = 0.0;             /*Deg*/
    public static final double ELEVATOR_AMP_POS = 0.0;              /*Deg*/
    
    public static final double PIVOT_STOW_POS = 0.0;                /*Deg*/
    public static final double PIVOT_AMP_POS = 30.0;                /*Deg*/
    public static final double PIVOT_INTAKE_POS = 45.0;             /*Deg*/

    public static final double INDEXER_SPEAKER_SPEED = 0.9;         /*Value*/
    public static final double INDEXER_INTAKE_SPEED = 0.5;          /*Value*/
    public static final double INDEXER_REVERSE_SPEED = -0.1;        /*value*/

    public static final double LEFT_SHOOTER_AMP_SPEED = 40.0;      /*Rot/s*/
    public static final double RIGHT_SHOOTER_AMP_SPEED = 40.0;     /*Rot/s*/
    public static final double INDEXER_AMP_SPEED = 0.2;             /*Value*/

    public static final double LEFT_SHOOTER_EJECT_SPEED = 40.0;     /*Rot/s*/
    public static final double RIGHT_SHOOTER_EJECT_SPEED = 40.0;    /*Rot/s*/
    public static final double INDEXER_EJECT_SPEED = -0.2;          /*Value*/

    public static final double INTAKE_SPEED = 0.6;                  /*Value*/
    public static final double INTAKE_REJECT_SPEED = -0.15;         /*Value*/
    private static final double INTAKE_EJECT_SPEED = -0.4;          /*value*/

    public static final double ELEVATOR_CLIMB_POS = 8.0;

    public static final double PIVOT_TIMEOUT = 1.0;                 /*sec*/
    public static final double ELEVATOR_TIMEOUT = 1.0;              /*sec*/
    public static final double TURRET_TIMEOUT = 1.0;                /*sec*/
    private static final double SHOOTER_TIMEOUT = 1.0;              /*sec*/
    private static final double AFTER_SHOOT_TIMEOUT = 0.2;          /*sec*/
    private static final double AFTER_AMP_SHOOT_TIMEOUT = 0.6; 

    private static final double TURRET_PREAIM_TIMEOUT = 0.5;        /*sec*/


    //TODO review values and code
    public static Command createEjectCommand(TurretSubsystem turret, IndexerSubsystem indexer, IntakeSubsystem intake) {
            return turret.newSetPosCommand(TURRET_STOW_POS)
                .andThen(turret.newWaitUntilSetpointCommand(TURRET_TIMEOUT))
                .andThen(indexer.newSetSpeedCommand(INDEXER_EJECT_SPEED))
                .andThen(intake.newSetSpeedCommand(INTAKE_EJECT_SPEED))
                .andThen(indexer.newUntilNoNoteFoundCommand()) // dont stop until note gone
                .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    //TODO review values and code
    public static Command createStopShootingCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, PivotSubsystem pivot,  TurretSubsystem turret) {
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .alongWith(indexer.newSetSpeedCommand(0.0))
            .alongWith(pivot.newSetPosCommand(PIVOT_STOW_POS))
            .alongWith(turret.newSetPosCommand(TURRET_STOW_POS));
    }
    
    public static Command createStopIntakingCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return intake.newSetSpeedCommand(0.0)
            .andThen(indexer.newSetSpeedCommand(0.0))
            .andThen(new InstantCommand(() -> 
                {LimelightHelpers.setLEDMode_ForceOff("limelight-front"); 
                LimelightHelpers.setLEDMode_ForceOff("limelight-back");}));
    }

    public static Command createNoteBackUpCommand(IndexerSubsystem indexer, IntakeSubsystem intake) {
        return new ConditionalCommand(
            indexer.newSetSpeedCommand(INDEXER_REVERSE_SPEED)
            .andThen(new WaitCommand(0.15))
            .andThen(indexer.newSetSpeedCommand(0.0)),
            new InstantCommand(),
            () -> indexer.getSensorDistance() >= 40)
                .andThen(CommandFactoryUtility.createStopIntakingCommand(intake, indexer))
                    .andThen(intake.newSetSpeedCommand(CommandFactoryUtility.INTAKE_REJECT_SPEED));
    }

    public static Command createRunIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, TurretSubsystem turret) {
        return indexer.newUntilNoNoteFoundCommand()  // make sure no note is found
            .andThen(turret.newSetPosCommand(TURRET_STOW_POS))
            .andThen(turret.newWaitUntilSetpointCommand(TURRET_TIMEOUT))
            .andThen(intake.newSetSpeedCommand(INTAKE_SPEED))
            .andThen(indexer.newSetSpeedCommand(INDEXER_INTAKE_SPEED))
            .andThen(indexer.newUntilNoteFoundCommand())
            .andThen(new WaitCommand(0.05))
            .alongWith(new InstantCommand(() -> 
                {LimelightHelpers.setLEDMode_ForceOn("limelight-front"); 
                LimelightHelpers.setLEDMode_ForceOn("limelight-back");})) // Wait on the intake, we're stopping too quickly
            // .andThen(createStopIntakingCommand(intake, indexer)) // currently used separately, only add if told
            .andThen(indexer.newSetSpeedCommand(0.0))
            .andThen(intake.newSetSpeedCommand(0.0)); // Dont stop intake until note found
    }

    public static Command createAmpScoreCommand(PivotSubsystem pivot, TurretSubsystem turret, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return //elevator.newSetPosCommand(ELEVATOR_AMP_POS)
                (pivot.newSetPosCommand(PIVOT_AMP_POS))
                    .andThen(turret.newSetPosCommand(TURRET_STOW_POS))
                    .andThen(shooter.newSetSpeedsCommand(LEFT_SHOOTER_AMP_SPEED, RIGHT_SHOOTER_AMP_SPEED))
                    //.andThen(elevator.newWaitUntilSetpointCommand(ELEVATOR_TIMEOUT)
                                .alongWith(pivot.newWaitUntilSetpointCommand(PIVOT_TIMEOUT))
                                .alongWith(turret.newWaitUntilSetpointCommand(TURRET_TIMEOUT))
                                .alongWith(shooter.newWaitUntilSetpointCommand(SHOOTER_TIMEOUT))
                    .andThen(indexer.newSetSpeedCommand(INDEXER_AMP_SPEED))
                    .andThen(indexer.newUntilNoNoteFoundCommand()) // dont stop until note gone
                    .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    // public static Command createElevatorClimbCommand(ElevatorSubsystem elevator) {
    //     return elevator.newSetPosCommand(ELEVATOR_CLIMB_POS)
    //         .andThen(elevator.newWaitUntilSetpointCommand(ELEVATOR_TIMEOUT));
    // }

    // public static Command createStowElevatorCommand(ElevatorSubsystem elevator) {
    //     return elevator.newSetPosCommand(ELEVATOR_STOW_POS);
    // }

    // TODO trap shot
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
    
    public static Command createSpeakerScoreCommand(SpeakerScoreUtility speakerUtil, ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, TurretSubsystem turret, Double pivotAngle, boolean adjustPivot) {
        Command command = null;
        if (adjustPivot){
            command = createPivotAndShooterSpeedCommand(shooter, pivot, pivotAngle);
        } else {
            command = new InstantCommand();
        }
        
        return command 
            .andThen(pivot.newWaitUntilSetpointCommand(PIVOT_TIMEOUT)
                .alongWith(shooter.newWaitUntilSetpointCommand(SHOOTER_TIMEOUT))
                )
            // .andThen(new TurretRefineCommand(turret).withTimeout(2.0))
            .andThen(indexer.newSetSpeedCommand(INDEXER_SPEAKER_SPEED))
            .andThen(indexer.newUntilNoNoteFoundCommand()) // dont stop until note gone
            .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    public static Command createSpeakerScoreCommand(SpeakerScoreUtility speakerUtil, ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, TurretSubsystem turret, Double pivotAngle) {
    {
        return createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret, pivotAngle, true);
    }}

    public static Command createSpeakerScoreCommand(SpeakerScoreUtility speakerUtil, ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, TurretSubsystem turret) {
        return createSpeakerScoreCommand(speakerUtil, shooter, pivot, indexer, turret, null);
    }

    public static Command createTurretPreaimCommand(TurretSubsystem turret) {
        //return new TurretAimCommand(turret)
        //    .raceWith(turret.newWaitUntilSetpointCommand(TURRET_PREAIM_TIMEOUT));
        return new InstantCommand();
    }

    public static Command createAmpShootCommand(AmpHoodSubsystem hood,ShooterSubsystem shooter,IndexerSubsystem indexer) {
        return 
        //hood.newWaitUntilAmpIsExtendedCommand().deadlineWith(hood.newExtendHoodCommand())
        shooter.newSetSpeedsCommand(LEFT_SHOOTER_AMP_SPEED, RIGHT_SHOOTER_AMP_SPEED)
        .andThen(shooter.newWaitUntilSetpointCommand(SHOOTER_TIMEOUT))
        .andThen(indexer.newSetSpeedCommand(INDEXER_AMP_SPEED))
        .andThen(indexer.newUntilNoNoteFoundCommand())
        .andThen(new WaitCommand(AFTER_AMP_SHOOT_TIMEOUT))
        .andThen(
            //hood.newRetractHoodCommand()
            shooter.newSetSpeedsCommand(0.0,0.0)
            .alongWith(indexer.newSetSpeedCommand(0))
        );
    }
}
