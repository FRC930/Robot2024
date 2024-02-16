package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.ShooterCommand;
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
    public static final double ELEVATOR_AMP_POS = 10.0;
    
    public static final double PIVOT_STOW_POS = 0.0;
    public static final double PIVOT_AMP_POS = 45.0;
    public static final double PIVOT_INTAKE_POS = 45.0;

    //#region SPEAKER SCORE CONSTANTS
    public static final double[] LEFT_SHOOTER_SPEAKER_SPEEDS = {0.85, 0.7, 0.7};
    public static final double[] RIGHT_SHOOTER_SPEAKER_SPEEDS = {0.85, 0.7, 0.7};
    public static final double INDEXER_SPEAKER_SPEED = 0.5;

    public static final double[] PIVOT_PIVOT_POSITIONS = {31.0, 33.0, 40.0};
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
    public static final double PIVOT_DEADBAND = 2.0;

    public static final double ELEVATOR_WAIT_TIME = 1.0;
    public static final double ELEVATOR_DEADBAND = 2.0;

    public static final double TURRET_WAIT_TIME = 1.0;
    public static final double TURRET_DEADBAND = 2.0;

    private static final double AFTER_SHOOT_TIMEOUT = 0.5;

    //TODO review values and code
    public static Command createEjectCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return new ShooterCommand(shooter,LEFT_SHOOTER_EJECT_SPEED, RIGHT_SHOOTER_EJECT_SPEED)
            .alongWith(new WaitCommand(1.0)
            .andThen(new IndexerCommand(indexer, INDEXER_EJECT_SPEED)));
    }

    //TODO review values and code
    public static Command createStopShootingCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .alongWith(indexer.newSetSpeedCommand(0.0));
    }

    public static Command createRunIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return new IntakeCommand(intake,INTAKE_SPEED)
            .alongWith(new IndexerCommand(indexer,INDEXER_INTAKE_SPEED))
            .until(() -> indexer.getSensor());// Ends intake when note is detected in indexer
    }

    public static Command createAmpScoreCommand(ElevatorSubsystem elevator, PivotSubsystem pivot, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return elevator.newSetPosCommand(ELEVATOR_AMP_POS)
                    .andThen(pivot.newSetPosCommand(PIVOT_AMP_POS))
                    .andThen(shooter.newSetSpeedsCommand(LEFT_SHOOTER_AMP_SPEED, RIGHT_SHOOTER_AMP_SPEED))
                    .andThen(elevator.newWaitUntilSetpointCommand(ELEVATOR_WAIT_TIME, ELEVATOR_DEADBAND)
                                .alongWith(pivot.newWaitUntilSetpointCommand(PIVOT_WAIT_TIME, PIVOT_DEADBAND))
                                // TODO: Another alongWith for shooter speed wait until
                                )
                    .andThen(new IndexerCommand(indexer, INDEXER_AMP_SPEED).until(() -> !indexer.getSensor()))
                    .andThen(new WaitCommand(AFTER_SHOOT_TIMEOUT)); // This is to validate that note is out
    }

    public static Command createElevatorClimbCommand(ElevatorSubsystem elevator) {
        return new SetElevatorPositionCommand(elevator, ELEVATOR_CLIMB_POS);
    }
}
