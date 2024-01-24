package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.sim.PhysicsSim;

public class ElevatorIORobot implements ElevatorIO {
    private final TalonFX leftElevatorFollower;
    private final TalonFX rightElevatorMaster;
    private final double gearRatio;
    private final double maxHeight;

    private final MotionMagicVoltage m_request;

    /**
     * <h3>ElevatorIORobot</h3> 
     * Creates a subsystem that represents an Elevator system
     * @param motorID The id of the elevator motor
     * @param config The PID and Feedforward controller configs
     * @param gearRatio The ratio of the motor rotations to the height on the elevator
     */
    public ElevatorIORobot (TalonFX motor1, TalonFX motor2, double gearRatio, double maxHeight, Slot0Configs config, MotionMagicConfigs mmConfigs){
        leftElevatorFollower = motor1;
        rightElevatorMaster = motor2;
        this.maxHeight = maxHeight;
        this.gearRatio = gearRatio;
       
        m_request = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config);
        cfg.Feedback.SensorToMechanismRatio = this.gearRatio; //The ratio between the motor turning and the elevator moving. We may have to invert this
        cfg.withMotionMagic(mmConfigs); // The Motion Magic configs

        leftElevatorFollower.getConfigurator().apply(cfg);
        rightElevatorMaster.getConfigurator().apply(cfg);
        leftElevatorFollower.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMaster.setNeutralMode(NeutralModeValue.Brake);

        leftElevatorFollower.setControl(new Follower(rightElevatorMaster.getDeviceID(), true));
        rightElevatorMaster.setControl(m_request.withPosition(0).withSlot(0));

        //TODO: TEMP
        if(Utils.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(leftElevatorFollower, 0.001);
            PhysicsSim.getInstance().addTalonFX(rightElevatorMaster, 0.001);
        }
    }
    

    @Override
    public void updateInputs() {}

    public void setTargetHeight(double height) {
        rightElevatorMaster.setControl(m_request.withPosition(MathUtil.clamp(height,0,maxHeight)).withSlot(0));
    }

    @Override
    public double getCurrentVelocity() {
        return rightElevatorMaster.getVelocity().getValue();
    }

    @Override
    public double getCurrentHeight() {
        return rightElevatorMaster.getPosition().getValue();
    }

    @Override
    public double getTargetHeight() {
        return ((MotionMagicVoltage) rightElevatorMaster.getAppliedControl()).Position;
    }  
}

