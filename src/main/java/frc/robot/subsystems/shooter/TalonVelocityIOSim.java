package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TalonVelocityIOSim extends TalonVelocityIORobot {
    private DCMotorSim m_motorSim;
    private final double kMotorResistance = 0.002;

    public TalonVelocityIOSim(int MotorID, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs) {
        super(MotorID, gearRatio, config, mmConfigs);
        m_motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), gearRatio, 0.001);
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
}
