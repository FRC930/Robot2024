package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class TrapAlignAndShootCommand extends Command {
    private enum Quadrant {
        RED_AMP(new Pose2d(9, 8, new Rotation2d()), new Pose2d(17, 4, new Rotation2d()),
                new Pose2d(11.72, 5.72, Rotation2d.fromDegrees(-120))),
        RED_NON_AMP(new Pose2d(9, 4, new Rotation2d()), new Pose2d(17, 0, new Rotation2d()),
                new Pose2d(12.8, 3.1, Rotation2d.fromDegrees(118))),
        BLUE_AMP(new Pose2d(0, 9, new Rotation2d()), new Pose2d(8, 4, new Rotation2d()),
                new Pose2d(3.64, 5, Rotation2d.fromDegrees(-55))),
        BLUE_NON_AMP(new Pose2d(0, 4, new Rotation2d()), new Pose2d(8, 0, new Rotation2d()),
                new Pose2d(4.63, 2.62, Rotation2d.fromDegrees(59)));

        public Pose2d start;
        public Pose2d end;
        public Pose2d shotPosition;

        Quadrant(Pose2d start, Pose2d end, Pose2d shotPosition) {
            this.start = start;
            this.end = end;
            this.shotPosition = shotPosition;
        }

        private static Quadrant fromStartingPosition(Alliance alliance, Pose2d startingPosition) {
            if (alliance == Alliance.Red) return fromRedStartingPosition(startingPosition);
            else return fromBlueStartingPosition(startingPosition);
        }

        private static Quadrant fromRedStartingPosition(Pose2d startingPosition) {
            Quadrant amp = Quadrant.RED_AMP;
            Quadrant nonAmp = Quadrant.RED_NON_AMP;

            if (inQuadrant(startingPosition, amp)) return amp;
            else if (inQuadrant(startingPosition, nonAmp)) return nonAmp;
            else return null;
        }

        private static Quadrant fromBlueStartingPosition(Pose2d startingPosition) {
            Quadrant amp = Quadrant.BLUE_AMP;
            Quadrant nonAmp = Quadrant.BLUE_NON_AMP;

            if (inQuadrant(startingPosition, amp)) return amp;
            else if (inQuadrant(startingPosition, nonAmp)) return nonAmp;
            else return null;
        }

        private static boolean inQuadrant(Pose2d startingPosition, Quadrant quadrant) {
            Pose2d start = quadrant.start;
            Pose2d end = quadrant.end;

            return start.getX() < startingPosition.getX() && start.getY() > startingPosition.getY()
                    && end.getX() > startingPosition.getX() && end.getY() < startingPosition.getY();

        }
    }

    private static Command alignToTrap(SwerveDrivetrainSubsystem drive) {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        Alliance alliance;
        if (optionalAlliance.isPresent()){
          alliance = optionalAlliance.get();
        } else {
          return new InstantCommand();
        }

        Pose2d startingPosition = drive.getState().Pose;
        Quadrant currentQuadrant = Quadrant.fromStartingPosition(alliance, startingPosition);

        if (currentQuadrant == null) return new InstantCommand();

        List<Translation2d> pathPoints = PathPlannerPath.bezierFromPoses(
            startingPosition,
            currentQuadrant.shotPosition
        );

        PathPlannerPath path = new PathPlannerPath(
            pathPoints, 
            new PathConstraints(3.0, 3.0, 2*Math.PI, 4*Math.PI), 
            new GoalEndState(0.0, currentQuadrant.shotPosition.getRotation())
        );

        path.preventFlipping = true;
        return AutoBuilder.followPath(path);
    }

    private SwerveDrivetrainSubsystem drive;
    private Command alignCommand;
    public TrapAlignAndShootCommand(SwerveDrivetrainSubsystem drive) {
        this.drive = drive;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        this.alignCommand = alignToTrap(this.drive);
        this.alignCommand.initialize();
    }

    @Override
    public void execute() {
        this.alignCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return this.alignCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && this.alignCommand != null) {
            this.alignCommand.end(true);
        }
        this.alignCommand = null;
    }
}
