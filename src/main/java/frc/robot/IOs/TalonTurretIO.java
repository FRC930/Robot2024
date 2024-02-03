package frc.robot.IOs;
/**
 * <h3>TalonTurretIO</h3>
 * IO for turret hardware/sim consistency
 */
public interface TalonTurretIO extends TalonRollerIO{
    /**
     * <h3>getDegrees</h3>
     * @return position of mechanism in degrees
     */
    public double getDegrees();
    /**
     * <h3>getMechRotations</h3>
     * @return rotations of mechanism
     */
    public double getMechRotations();
    /**
     * <h3>setVoltage</h3>
     * Sets voltage of turret motor for direct control
     * @param volts amount of volts to apply
     */
    public void setVoltage(double volts);
}
