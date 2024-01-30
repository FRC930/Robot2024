package frc.robot.subsystems.roller;

import com.ctre.phoenix6.hardware.TalonFX;

public interface RollerMotorIO {
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