package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.TurretLimeLightAimCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public final class CommandFactoryUtility {

    //#region positions
    public static final double TURRET_STOW_POS = TurretSubsystem.STOW_POS;

    public static final double ELEVATOR_STOW_POS = 0.0;
    public static final double ELEVATOR_AMP_POS = 8.0;
    
    public static final double PIVOT_STOW_POS = 0.0;
    public static final double PIVOT_AMP_POS = 45.0;
    public static final double PIVOT_INTAKE_POS = 45.0;

    //#region SPEAKER SCORE CONSTANTS
    public static final double INDEXER_SPEAKER_SPEED = 0.9;
    //#endregion

    public static final double LEFT_SHOOTER_AMP_SPEED = -0.3;
    public static final double RIGHT_SHOOTER_AMP_SPEED = -0.3;
    public static final double INDEXER_AMP_SPEED = 0.2;

    public static final double LEFT_SHOOTER_EJECT_SPEED = 0.2;
    public static final double RIGHT_SHOOTER_EJECT_SPEED = 0.2;
    public static final double INDEXER_EJECT_SPEED = 0.2;

    public static final double INTAKE_SPEED = 0.6;
    public static final double INTAKE_REJECT_SPEED = -0.15;

    public static final double INDEXER_INTAKE_SPEED = 0.2;

    public static final double ELEVATOR_CLIMB_POS = 8.0;

    public static final double PIVOT_WAIT_TIME = 1.0;

    public static final double ELEVATOR_WAIT_TIME = 1.0;

    public static final double TURRET_WAIT_TIME = 1.0;

    private static final double SHOOTER_WAIT_TIME = 1.0;

    private static final double AFTER_SHOOT_TIMEOUT = 0.5;

    //TODO review values and code
    public static Command createEjectCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(LEFT_SHOOTER_EJECT_SPEED, RIGHT_SHOOTER_EJECT_SPEED)
            .andThen(new WaitCommand(1.0)) // TODO wait til shooterspeed
            .andThen(indexer.newSetSpeedCommand(INDEXER_EJECT_SPEED))
            .andThen(indexer.newUnlessNoteFoundCommand()); // dont stop until note gone
    }

    //TODO review values and code
    public static Command createStopShootingCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, PivotSubsystem pivot, ElevatorSubsystem evelator) {
        // TODO Need to reset turret?
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .alongWith(indexer.newSetSpeedCommand(0.0))
            .alongWith(evelator.newSetPosCommand(0.0))
            .alongWith(pivot.newSetPosCommand(PIVOT_STOW_POS));
    }

    public static Command createRunIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, TurretSubsystem turret) {
        return indexer.newUnlessNoteFoundCommand()  // make sure no note is found
            .andThen(turret.newSetPosCommand(TURRET_STOW_POS))
            .until(() -> turret.atSetpoint())
            .andThen(intake.newSetSpeedCommand(INTAKE_SPEED))
            .andThen(indexer.newSetSpeedCommand(INDEXER_INTAKE_SPEED))
            .andThen(indexer.newUntilNoteFoundCommand()); // Dont stop intake until note found
    }

    public static Command createAmpScoreCommand(ElevatorSubsystem elevator, PivotSubsystem pivot, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return elevator.newSetPosCommand(ELEVATOR_AMP_POS)
                    .andThen(pivot.newSetPosCommand(PIVOT_AMP_POS))
                    .andThen(shooter.newSetSpeedsCommand(LEFT_SHOOTER_AMP_SPEED, RIGHT_SHOOTER_AMP_SPEED))
                    .andThen(elevator.newWaitUntilSetpointCommand(ELEVATOR_WAIT_TIME)
                                .alongWith(pivot.newWaitUntilSetpointCommand(PIVOT_WAIT_TIME))
                                .alongWith(shooter.newWaitUntilSetpointCommand(SHOOTER_WAIT_TIME))
                                )
                    .andThen(indexer.newSetSpeedCommand(INDEXER_AMP_SPEED))
                    .andThen(indexer.newUnlessNoteFoundCommand()) // dont stop until note gone
                    .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    

    public static Command createElevatorClimbCommand(ElevatorSubsystem elevator) {
        return new SetElevatorPositionCommand(elevator, ELEVATOR_CLIMB_POS);
    }

    public static Command createStowElevatorCommand(ElevatorSubsystem elevator) {
        return new SetElevatorPositionCommand(elevator, ELEVATOR_STOW_POS);
    }

    // TODO trap shot
    public static Command createSpeakerScoreCommand(SpeakerScoreUtility speakerUtil, ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, TurretSubsystem turret) {
        return new TurretLimeLightAimCommand(turret).withTimeout(.2) // TODO does not command does not end???
            .andThen(shooter.newSetSpeedsCommand(speakerUtil))
            .andThen(pivot.newSetPosCommand(speakerUtil))
            .andThen(pivot.newWaitUntilSetpointCommand(PIVOT_WAIT_TIME)
                    .alongWith(shooter.newWaitUntilSetpointCommand(SHOOTER_WAIT_TIME))
                    )
            .andThen(indexer.newSetSpeedCommand(INDEXER_SPEAKER_SPEED))
            .andThen(indexer.newUnlessNoteFoundCommand()) // dont stop until note gone
            .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

}
