package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * <h3>ElevatorIOSim</h3>
 * An IO to the simulated robot's elevator
 */
public class ElevatorIOSim extends ElevatorIORobot{
    //private ElevatorSim elevatorSim;
    private final double kMotorResistance = 0.002;
    private DCMotorSim m_motorSim;
    /**
     * <h3>ElevatorIOSim</h3> 
     * Creates a subsystem that represents an Elevator system
     * @param motorID The id of the elevator motor
     * @param config The PID and Feedforward controller configs
     * @param gearRatio The ratio of the motor rotations to the height on the elevator
     */
    public ElevatorIOSim ( //TODO elevator simulation is currently not working
        int motor1ID, 
        int motor2ID, 
        String canbus,
        Slot0Configs config, 
        MotionMagicConfigs mmConfigs,
        ElevatorType elevator
        ){
        super(motor1ID,motor2ID,canbus,config,mmConfigs,elevator);
        //this.elevatorSim = new ElevatorSim(elevator.m_kV, elevator.m_kA, DCMotor.getKrakenX60Foc(2), elevator.m_minHeight, elevator.m_maxHeight, true, elevator.m_startingHeight);
        m_motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), elevator.m_gearRatio, 0.001);
    }

    @Override
    public void runSim() {
        /// DEVICE SPEED SIMULATION
    
        m_motorSim.setInputVoltage(rightElevatorMaster.getSimState().getMotorVoltage());
    
        m_motorSim.update(getPeriod());
    
        /// SET SIM PHYSICS INPUTS
        final double position_rot = m_motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(m_motorSim.getAngularVelocityRadPerSec());
    
        rightElevatorMaster.getSimState().setRawRotorPosition(position_rot);
        rightElevatorMaster.getSimState().setRotorVelocity(velocity_rps);
    
        rightElevatorMaster.getSimState().setSupplyVoltage(12 - rightElevatorMaster.getSimState().getSupplyCurrent() * kMotorResistance);
    }
}
