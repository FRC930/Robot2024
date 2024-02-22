package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.TurretRefineCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.mm_turret.mmTurretSubsystem;

public final class CommandFactoryUtility {

    //#region positions
    public static final double TURRET_STOW_POS = 0.0;

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

    public static final double INTAKE_SPEED = 0.8;
    public static final double INTAKE_REJECT_SPEED = -0.15;

    public static final double INDEXER_INTAKE_SPEED = 0.8;

    public static final double ELEVATOR_CLIMB_POS = 8.0;

    public static final double PIVOT_TIMEOUT = 1.0;

    public static final double ELEVATOR_TIMEOUT = 1.0;

    public static final double TURRET_TIMEOUT = 1.0;

    private static final double SHOOTER_TIMEOUT = 1.0;

    private static final double AFTER_SHOOT_TIMEOUT = 2.0;

    //TODO review values and code
    public static Command createEjectCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(LEFT_SHOOTER_EJECT_SPEED, RIGHT_SHOOTER_EJECT_SPEED)
            .andThen(shooter.newWaitUntilSetpointCommand(SHOOTER_TIMEOUT))
            .andThen(indexer.newSetSpeedCommand(INDEXER_EJECT_SPEED))
            .andThen(indexer.newUnlessNoteFoundCommand()) // dont stop until note gone
            .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT));
    }

    //TODO review values and code
    public static Command createStopShootingCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, PivotSubsystem pivot, ElevatorSubsystem elevator) {
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .alongWith(elevator.newSetPosCommand(ELEVATOR_STOW_POS))
            .alongWith(pivot.newSetPosCommand(PIVOT_STOW_POS));
    }

    public static Command createRunIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, mmTurretSubsystem turret) {
        return indexer.newUnlessNoteFoundCommand()  // make sure no note is found
            .andThen(turret.newSetPosCommand(TURRET_STOW_POS))
            .andThen(turret.newWaitUntilSetpointCommand(TURRET_TIMEOUT))
            .andThen(intake.newSetSpeedCommand(INTAKE_SPEED))
            .andThen(indexer.newSetSpeedCommand(INDEXER_INTAKE_SPEED))
            .andThen(indexer.newUntilNoteFoundCommand())
            .andThen(intake.newSetSpeedCommand(0.0))
            .andThen(indexer.newSetSpeedCommand(0.0)); // Dont stop intake until note found
    }

    public static Command createAmpScoreCommand(ElevatorSubsystem elevator, PivotSubsystem pivot, mmTurretSubsystem turret, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return elevator.newSetPosCommand(ELEVATOR_AMP_POS)
                    .andThen(pivot.newSetPosCommand(PIVOT_AMP_POS))
                    .andThen(turret.newSetPosCommand(TURRET_STOW_POS))
                    .andThen(shooter.newSetSpeedsCommand(LEFT_SHOOTER_AMP_SPEED, RIGHT_SHOOTER_AMP_SPEED))
                    .andThen(elevator.newWaitUntilSetpointCommand(ELEVATOR_TIMEOUT)
                                .alongWith(pivot.newWaitUntilSetpointCommand(PIVOT_TIMEOUT))
                                .alongWith(turret.newWaitUntilSetpointCommand(TURRET_TIMEOUT))
                                .alongWith(shooter.newWaitUntilSetpointCommand(SHOOTER_TIMEOUT))
                                )
                    .andThen(indexer.newSetSpeedCommand(INDEXER_AMP_SPEED))
                    .andThen(indexer.newUnlessNoteFoundCommand()) // dont stop until note gone
                    .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    public static Command createElevatorClimbCommand(ElevatorSubsystem elevator) {
        return elevator.newSetPosCommand(ELEVATOR_CLIMB_POS)
            .andThen(elevator.newWaitUntilSetpointCommand(ELEVATOR_TIMEOUT));
    }

    public static Command createStowElevatorCommand(ElevatorSubsystem elevator) {
        return elevator.newSetPosCommand(ELEVATOR_STOW_POS);
    }

    // TODO trap shot
    public static Command createSpeakerScoreCommand(SpeakerScoreUtility speakerUtil, ShooterSubsystem shooter, PivotSubsystem pivot, IndexerSubsystem indexer, mmTurretSubsystem turret) {
        return shooter.newCalcAndSetSpeedsCommand() //shooter.newSetSpeedsCommand(speakerUtil)
            .andThen(pivot.newCalcAndSetPosCommand()) //.andThen(pivot.newSetPosCommand(speakerUtil))
            .andThen(pivot.newWaitUntilSetpointCommand(PIVOT_TIMEOUT)
                .alongWith(shooter.newWaitUntilSetpointCommand(SHOOTER_TIMEOUT))
                )
            .andThen(new TurretRefineCommand(turret).withTimeout(2.0))
            .andThen(indexer.newSetSpeedCommand(INDEXER_SPEAKER_SPEED))
            .andThen(indexer.newUnlessNoteFoundCommand()) // dont stop until note gone
            .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

}
