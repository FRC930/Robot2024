package frc.robot.subsystems.mm_turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * <h3>PivotIO</h3>
 * An IO to the simulated robot's pivot
 */
public class mmTurretIOSim extends mmTurretIORobot {

    // private SingleJointedArmSim m_ArmSim;

    private final double kMotorResistance = 0.002;
    // private final double armMOI = 0.001;

	private DCMotorSim m_motorSim;
    
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     * @param motorID The id of the pivot motor
     */
    public mmTurretIOSim(int id, int encoderID, String canbus, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs, double encoderOffset) {
        super(id, encoderID, canbus, gearRatio, config, mmConfigs,encoderOffset);
        // m_ArmSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(0), gearRatio, gearRatio, gearRatio, gearRatio, gearRatio, false, gearRatio);
        // TODO Simulation appear to work better with gearratio = 1
        m_motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1,0.001);
    }
    
    // @Override
    // public void runSim() {
    //     m_ArmSim.setInputVoltage(m_motor.getSimState().getMotorVoltage());

    //     m_ArmSim.update(0.02);
        
    //     /// SET SIM PHYSICS INPUTS
    //     final double position_rot = Units.radiansToRotations(m_ArmSim.getAngleRads());
    //     final double velocity_rps = Units.radiansToRotations(m_ArmSim.getVelocityRadPerSec());

    //     m_motor.getSimState().setRawRotorPosition(position_rot);
    //     m_motor.getSimState().setRotorVelocity(velocity_rps);

    //     m_motor.getSimState().setSupplyVoltage(12 - m_motor.getSimState().getSupplyCurrent() * kMotorResistance);
    // }
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
}
