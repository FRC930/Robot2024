package frc.robot.IOs;

import com.ctre.phoenix6.hardware.TalonFX;

public interface TalonRollerIO {
    /**
     * <h3>setSpeed</h3>
     * Sets speed based off of percent output
     * @param speed - 
     */
    public void setSpeed(double speed);
    public double getSpeed();
    public double getVoltage();
    /**
     * <h3>updateInputs</h3>
     * Steps the simulation forward by 0.02 seconds.
     * <p>Does nothing on physical robot
     */
    public void runSim();
    public TalonFX getTalon();
    /**
     * <h3>getPeriod</h3>
     * @return
     */
    default double getPeriod() {
        return 0.02;
    }
}