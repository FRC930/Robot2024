package frc.robot.subsystems.elevator;

/**
 * <h3>ElevatorIO</h3>
 * 
 * Sets up the mothods that we use in the IORobot and IOSim
 */

public interface ElevatorIO {
    /**
     * <h3>updateInputs</h3>
     * Steps the simulation forward by 0.02 seconds.
     * <p>Does nothing on physical robot
     */
    public void updateInputs();
    /**
     * <h3>setVoltage</h3>
     * Sets the current voltage of the elevator
     * @param volts
     */
    public void setVoltage(double volts);
    /**
     * <h3>getCurrentVelocity</h3>
     * Gets the current velocity of the elevator
     * @return The velocity of the elevator
     */
    public double getCurrentVelocity();
    /**
     * <h3>getCurrentHeight</h3>
     * Gets the current height of the elevator
     * @return The height of the elevator
     */
    public double getCurrentHeight();
}
