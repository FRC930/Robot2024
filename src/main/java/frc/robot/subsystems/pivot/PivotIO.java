package frc.robot.subsystems.pivot;

public interface PivotIO {
    /**
     * <h3>updateInputs</h3>
     * Steps the simulation forward by 0.02 seconds.
     * <p>Does nothing on physical robot
     */
    public void updateInputs();
    /**
     * <h3>getCurrentAngleDegrees</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from the horizontal
     */
    public double getCurrentAngleDegrees();
     /**
     * <h3>getVelocityDegreesPerSecond</h3>
     * Gets the angular velocity of the turret.
     * @return The velocity in degrees per second where 0 is the horizontal and positive is up.
     */
    public double getVelocityDegreesPerSecond();
    /**
     * <h3>setVoltage</h3>
     * Sets the voltage output of the turret motor.
     * <p><b>THIS DOES NOT CLAMP THE VOLTAGE, BE CAREFUL WHAT YOU INPUT</b>
     * @param volts The voltage
     */
    public void setVoltage(double volts);
    /**
     * <h3>adjustOffsetDegrees</h3>
     * Sets the degrees from the horizontal that the turret is offset.
     * @param offsetDegrees
     */
    public void adjustOffsetDegrees(double offsetDegrees);
    /**
     * <h3>getVoltage</h3>
     * Gets the voltage of the motor.
     */
    public double getVoltage();
}
