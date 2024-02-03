package frc.robot.IOs;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * <h3>TalonRollerIO</h3>
 * IO meant for Talons that are set with duty cycle, not position-based
 */
public interface TalonRollerIO {
    /**
     * <h3>setSpeed</h3>
     * Sets speed based off of percent output
     * @param speed - 
     */
    public void setSpeed(double speed);
    /**
     * <h3>getSpeed</h3>
     * Gets the current speed of the mechanism
     * @return The current speed
     */
    public double getSpeed();
    /**
     * <h3>getVoltage</h3>
     * Gets the current voltage of the subystem.
     * @return The voltage the motor is running at.
     */
    public double getVoltage();
    /**
     * <h3>updateInputs</h3>
     * Steps the simulation forward by 0.02 seconds.
     * <p>Does nothing on physical robot
     */
    public void runSim();
    /**
     * <h1>getTalon</h1> 
     * @return the talonFX controller of this roller.
     */
    public TalonFX getTalon();
    /**
     * <h3>getPeriod</h3>
     * @return The period of this roller
     */
    default double getPeriod() {
        return 0.02;
    }
}