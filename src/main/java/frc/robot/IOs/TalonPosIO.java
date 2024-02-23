package frc.robot.IOs;

/**
 * <h3>TalonPosIO</h3>
 * IO for Talons that are set with positions
 */
public interface TalonPosIO {
    /**
     * <h3>runSim</h3>
     * Steps the simulation forward by 0.02 seconds.
     * <p>Does nothing on physical robot
     */
    public void runSim();
    /**
     * <h3>getPos</h3>
     * Gets the current position of the mechanism.
     * @return The current position
     */
    public double getPos();
     /**
     * <h3>getVelocity</h3>
     * Gets the current velocity of the mechanism
     * @return The current velocity
     */
    public double getVelocity();
    /**
     * <h3>getVoltage</h3>
     * Gets the current voltage of the subystem.
     * @return The voltage the motor is running at.
     */
    public double getVoltage();
    /**
     * <h3>getTarget</h3>
     * Gets the position that the mechanism is moving towards
     * @param angle The target 
     */
    public double getTarget();
    /**
     * <h3>setTarget</h3>
     * Sets the position that the mechanism is moving towards
     * @param position The target
     */
    public void setTarget(double position);
    /**
     * <h3>setPull</h3>
     * Sets the position that the mechanism is moving towards, uses different PID's
     * @param position The target
     */
    public void setRefinedTarget(double position);
    /**
     * <h3>getPeriod</h3>
     * @return
     */
    default double getPeriod() {
        return 0.02;
    }
    public void delayedConfigure();
}
