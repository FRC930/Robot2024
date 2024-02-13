package frc.robot.IOs;

/**
 * <h3>TimeOfFlightIO</h3>
 * An interface that represents a Time Of Flight sensor.
 */
public interface TimeOfFlightIO {
    /**
     * <h3>get</h3>
     * @return value of sensor
     */
    public boolean get();
    /**
     * <h3>getRange</h3>
     * @return detected range in milimeters if robot is real, otherwise, returns -1
     */
    public double getRange();
}