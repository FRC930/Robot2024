package frc.robot.utilities;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface MotorIO extends MotorController{
    /**
     * <h3>resetToFactoryDefaults</h3>
     * Resets the motors settings to its factory defaults.
     */
    public void resetToFactoryDefaults();

    /**
     * <h3>setShouldBrake</h3>
     * Sets whether the motor should brake or coast when stopped.
     */
    public void setShouldBrake(boolean shouldBrake);

    /**
     * <h3>getMaxVoltage</h3>
     * @return The maximum operating voltage of the motor
     */
    public double getMaxVoltage();

    /**
     * <h3>getIOVelocity</h3>
     * @return The current velocity that the motor encoder is reading in radians/second.
     */
    public double getIOVelocity();  

    /**
     * <h3>getIOPosition</h3>
     * @return The current position that the motor encoder is reading in radians.
     */
    public double getIOPosition();  
}   

