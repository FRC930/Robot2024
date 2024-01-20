package frc.robot.subsystems.pivot;


import edu.wpi.first.units.Units;
import frc.robot.utilities.TalonWrapper;


public class PivotIORobot implements PivotIO{
    private TalonWrapper m_motor;

    private double offsetDegrees;
    
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     * @param motorID The id of the pivot motor
     */
    public PivotIORobot(TalonWrapper io) {
        m_motor = io;
        offsetDegrees = 0.0;
    }
    
    @Override
    public double getVelocityDegreesPerSecond() {
       return Units.DegreesPerSecond.convertFrom(m_motor.getIOVelocity(), Units.RadiansPerSecond); // TODO: make sure this is right direction
    } 

    @Override
    public void updateInputs() {}

    @Override
    public double getCurrentAngleDegrees() {
        return Units.Degrees.convertFrom(m_motor.getIOPosition(),Units.Radians) + offsetDegrees; // TODO: make sure this is right direction
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void adjustOffsetDegrees(double offsetDegrees) {
        this.offsetDegrees = offsetDegrees;
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValue();
    }
}
