package frc.robot.subsystems.elevator;

/**
 * <h3>ElevatorIO</h3>
 * 
 * Sets up the mothods that we use in the IORobot and IOSim
 */

public interface ElevatorIO {
    public void updateInputs();
    public void setVoltage(double volts);
    public double getCurrentVelocity();
    public double getCurrentHeight();
}
