package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * <h3>TurretIOSim</h3>
 * An IO to the simulated robot's turret
 */
public class TurretIOSim extends TurretIORobot {

    // private SingleJointedArmSim m_ArmSim;

    private final double kMotorResistance = 0.002;
    // private final double armMOI = 0.001;

	private DCMotorSim m_motorSim;
    private double m_offset;
    private double m_gearRatio;
    
    /*
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     * @param motorID The id of the pivot motor
     */
    public TurretIOSim(int motorID, int encoderID, String canbus, double gearRatio, double offset) {
        super(motorID, encoderID, canbus, gearRatio, offset);
        // m_ArmSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(0), gearRatio, gearRatio, gearRatio, gearRatio, gearRatio, false, gearRatio);
        m_motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), gearRatio ,0.001);
        m_gearRatio = gearRatio;
        m_offset = offset;
    }

    @Override
    public void runSim() {
        /// DEVICE SPEED SIMULATION
    
        m_motorSim.setInputVoltage(m_motor.getSimState().getMotorVoltage());
    
        m_motorSim.update(getPeriod());
    
        /// SET SIM PHYSICS INPUTS
        final double position_rot = m_motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(m_motorSim.getAngularVelocityRadPerSec());
    
        m_motor.getSimState().setRawRotorPosition(position_rot);
        m_motor.getSimState().setRotorVelocity(velocity_rps);
    
        m_motor.getSimState().setSupplyVoltage(12 - m_motor.getSimState().getSupplyCurrent() * kMotorResistance);
    }

    @Override
    public double getMechRotations() {
        return ((Units.radiansToRotations(m_motorSim.getAngularPositionRad()) / m_gearRatio) % 1.0) + Units.degreesToRotations(m_offset); // Mod rotations by 1 to simulate absolute encoder
    }
}
