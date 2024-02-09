package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TalonVelocityIOSim extends TalonVelocityIORobot {
    private DCMotorSim m_motorSim;
    private final double kMotorResistance = 0.002;
    
    private MotionMagicVelocityVoltage m_simRequest;

    public TalonVelocityIOSim(int MotorID, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs) {
        super(MotorID, gearRatio, config, mmConfigs,false, new MotionMagicVelocityVoltage(0,0,true,0,0,false,false,false));
        m_motorSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), gearRatio, 0.001);
        m_simRequest = (MotionMagicVelocityVoltage) m_motor.getAppliedControl();
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

    /**
    * <h3>setSpeed</h3>
    * @param speed the speed the wheel will be set to
    */
    @Override
    public void setSpeed(double speed,double acceleration) {
        m_motor.setControl(m_simRequest.withVelocity(speed).withSlot(0));
    }

    /**
    * <h3>getTargetVelocity</h3>
    * @return The current voltage of the right motor
    */
    @Override
    public double getTargetVelocity() {
        return ((MotionMagicVelocityVoltage) m_motor.getAppliedControl()).Velocity;
    }
}
