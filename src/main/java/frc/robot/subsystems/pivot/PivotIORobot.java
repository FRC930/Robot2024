package frc.robot.subsystems.pivot;


import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utilities.constants.MotorConstants;


public class PivotIORobot implements PivotIO{
    private TalonFX m_motor;

    private double offsetDegrees;
    
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     * @param motorID The id of the pivot motor
     */
    public PivotIORobot(int motorID) {
        m_motor = new TalonFX(motorID);
        MotorConstants.resetTalonFX(m_motor,true);
    }
    
    @Override
    public double getVelocityDegreesPerSecond() {
       return m_motor.getVelocity().getValue() * 360; // TODO: make sure this is right direction
    } 

    @Override
    public void updateInputs() {}

    @Override
    public double getCurrentAngleDegrees() {
        return m_motor.getPosition().getValue() * 360 + offsetDegrees; // TODO: make sure this is right direction
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.set(volts);
    }

    @Override
    public void adjustOffsetDegrees(double offsetDegrees) {
        this.offsetDegrees = offsetDegrees;
    }
}
