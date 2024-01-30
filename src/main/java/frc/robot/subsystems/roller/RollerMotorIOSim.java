package frc.robot.subsystems.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class RollerMotorIOSim extends RollerMotorIOReal {

    private DCMotorSim m_sim;
    private final double kMotorResistance = 0.002;

    public RollerMotorIOSim(int id) {
        super(id);
        m_sim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.0,0.001);
    }

    @Override
    public void runSim() {
        /// DEVICE SPEED SIMULATION
    
        m_sim.setInputVoltage(m_motor.getSimState().getMotorVoltage());
    
        m_sim.update(getPeriod());
    
        /// SET SIM PHYSICS INPUTS
        final double position_rot = m_sim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(m_sim.getAngularVelocityRadPerSec());
    
        m_motor.getSimState().setRawRotorPosition(position_rot);
        m_motor.getSimState().setRotorVelocity(velocity_rps);
    
        m_motor.getSimState().setSupplyVoltage(12 - m_motor.getSimState().getSupplyCurrent() * kMotorResistance);
    }
}