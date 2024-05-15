package frc.robot.utilities;

/**
 * 
 * A utility class for the Lime Light cameras
 * 
 */
public class LimeLightDetectionUtility {

    public String m_LimeLightName;

    /**
     * 
     * A utility class for the Lime Light cameras
     * 
     * @param LimeLightName The name of the camera
     * 
     */
    public LimeLightDetectionUtility(String LimeLightName) {
        m_LimeLightName = LimeLightName;
    }

    public double get_tx() { //returns the amount of degrees off horizontally an object is from the crosshair of the camera
        return LimelightHelpers.getTX(m_LimeLightName); 
    }

    public double get_ty() { //returns the amount of degrees off vertically an object is from the crosshair of the camera
        return LimelightHelpers.getTY(m_LimeLightName);
    }

    public double get_ta() { //returns the percentage of the screen the object takes up
        return LimelightHelpers.getTA(m_LimeLightName);
    }
}