package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

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
     * <h3>getVoltage</h3>
     * Gets the current voltage of the subystem.
     * @return The voltage the motor is running at.
     */
    public double getVoltage();
    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getSetPoint();
    /**
     * <h3>setPosition</h3>
     * Sets the degrees from the horizontal that the turret will move to.
     * @param offsetDegrees
     */
    public void setPosition(double position);
}
