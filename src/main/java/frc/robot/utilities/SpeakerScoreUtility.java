package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpeakerScoreUtility {

    // {LeftShooterSpeed, RightShooterSpeed, IndexerSpeed, PivotPosition, ElevatorPosition}
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
}