package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpeakerScoreUtility {

    // {LeftShooterSpeed, RightShooterSpeed, IndexerSpeed, PivotPosition, ElevatorPosition}
    static final private int LEFT_SPEED_COLUMN = 0;
    static final private int RIGHT_SPEED_COLUMN = 1;
    static final private int INDEXER_SPEED_COLUMN = 2;  // TODO not needed since never used different value
    static final private int PIVOT_ANGLE_SPEED_COLUMN = 3;
    static final private int EVELATOR_HEIGHT_COLUMN = 4;   // TODO not need given all at 0.0 

    static final private int CLOSE_ROW = 0;
    static final private int MEDIUM_ROW = 1;
    static final private int FAR_ROW = 2;
    private final double[][] SHOOTING_CONSTANTS = 
    {
        {0.7, 0.7, 0.9, 40.0, 0.0}, // "close" 3' 7" from speaker bumper to front of frame
        {0.7, 0.7, 0.9, 33.0, 0.0}, // "medium" 7' 8" from speaker bumper to front of frame
        {0.85, 0.85, 0.9, 31.0, 0.0} // "far" 12' 7" from speaker bumper to front of frame (ONLY WORKS WITH UNTRIMMED NOTES)
    };


    public enum Target{
        close, // 3' 7" from speaker bumper to front of frame
        medium, // 7' 8" from speaker bumper to front of frame
        far // 12' 7" from speaker bumper to front of frame (ONLY WORKS WITH UNTRIMMED NOTES)
    };

    private Target m_desiredTarget;

    public SpeakerScoreUtility() {
        m_desiredTarget = Target.close;
    }

    public void setDesiredTarget(Target target) {
        m_desiredTarget = target;
        SmartDashboard.putString(this.getClass().getSimpleName()+"/DesiredTarget", target.toString());
    }

    public Target getDesiredTarget() {
        return m_desiredTarget;
    }

    public boolean isClose() {
        return m_desiredTarget == Target.close;
    }

    public boolean isMedium() {
        return m_desiredTarget == Target.medium;
    }

    public boolean isFar() {
        return m_desiredTarget == Target.far;
    } 

    public Command setDesiredTargetCommand(Target desiredTarget) {
        return new InstantCommand(() -> setDesiredTarget(desiredTarget));
    }

    public double getLeftShooterSpeed() {                
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][LEFT_SPEED_COLUMN];
    }

    public double getRightShooterSpeed() {
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][RIGHT_SPEED_COLUMN];
    }

    public double getPivotAngle() {
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][PIVOT_ANGLE_SPEED_COLUMN];
    }

    private int getRowForDesiredTarget() {
        switch(m_desiredTarget) {
            case close:
                return CLOSE_ROW;
            case medium:
                return MEDIUM_ROW;
            case far:
                return FAR_ROW;
            default:
                return CLOSE_ROW;
        }
    }

}