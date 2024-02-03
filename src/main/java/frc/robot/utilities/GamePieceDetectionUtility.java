package frc.robot.utilities;

import org.littletonrobotics.junction.Logger;

/**
 * 
 * A utility class for the game piece detection Lime Light camera
 * 
 */
public class GamePieceDetectionUtility {

    private String m_LimeLightName;

    /**
     * 
     * A utility class for the game piece detection Lime Light camera
     * 
     * @param LimeLightName The name of the camera1
     * 
     */
    public GamePieceDetectionUtility(String LimeLightName) {
        m_LimeLightName = LimeLightName;
        LimelightHelpers.setPipelineIndex(m_LimeLightName, 1);
    }

    public double get_tx() { //returns the amount of degrees off horizontally a game piece is from the center of the camera
        return LimelightHelpers.getTX(m_LimeLightName); 
    }

    public double get_ty() { //returns the amount of degrees off vertically a game piece is from the center of the camera
        return LimelightHelpers.getTY(m_LimeLightName);
    }

    public double get_ta() { //returns the percentage of the screen the game piece takes up
        return LimelightHelpers.getTA(m_LimeLightName);
    }
}





