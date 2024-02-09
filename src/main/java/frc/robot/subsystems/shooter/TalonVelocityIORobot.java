package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.IOs.TalonVelocityIO;

public class TalonVelocityIORobot implements TalonVelocityIO{
    private double gearRatio;

    protected TalonFX m_motor; 

    protected MotionMagicVelocityTorqueCurrentFOC m_request;

    protected TalonVelocityIORobot(int MotorID, double gearRatio,Slot0Configs config, MotionMagicConfigs mmConfigs, boolean initReal, MotionMagicVelocityVoltage simRequest) {
        m_motor = new TalonFX(MotorID); //Initializes the motor

        this.gearRatio = gearRatio; // The gear ratio
        
        
        m_request = new MotionMagicVelocityTorqueCurrentFOC(0,0,true,0,0,true,false,false); //The request that will be sent to the motor

        

        TalonFXConfiguration cfg = new TalonFXConfiguration(); //Creates a new blank TalonFX congfiguration that will be applied to the motors in a bit
        cfg.withSlot0(config); // The PID and FF configs
        cfg.Feedback.SensorToMechanismRatio = this.gearRatio; //The ratio between the motor turning and the elevator moving. We may have to invert this
        cfg.withMotionMagic(mmConfigs); // The Motion Magic configs

        // cfg.Voltage.PeakForwardVoltage = 8;
        // cfg.Voltage.PeakReverseVoltage = -8;
        // cfg.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        // cfg.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        m_motor.getConfigurator().apply(cfg); //Applies the configuration to the motor

        m_motor.setNeutralMode(NeutralModeValue.Coast); //Makes the motor continue rotating even when it is told to brake (its velocity is set to 0)

        m_motor.setControl(m_request.withVelocity(0).withSlot(0));

        // if(initReal) {
        //     m_motor.setControl(m_request.withVelocity(0).withSlot(0));
        // } else {
        //     m_motor.setControl(simRequest);
        // }
        
    }

    public TalonVelocityIORobot(int MotorID, double gearRatio,Slot0Configs config, MotionMagicConfigs mmConfigs) {
        this(MotorID, gearRatio,config,  mmConfigs, true, null);
    }

    /**
    * <h3>setSpeed</h3>
    * @param speed the speed the wheel will be set to
    */
    @Override
    public void setSpeed(double speed,double acceleration) {
        m_motor.setControl(m_request.withVelocity(speed).withAcceleration(acceleration).withSlot(0));
    }
    /**
    * <h3>setSpeed</h3>
    * @param speed the speed the wheel will be set to
    */
    @Override
    public void setSpeed(double speed) {
        m_motor.setControl(m_request.withVelocity(speed).withSlot(0));
    }

    /**
    * <h3>geMotorSpeed</h3>
    * @return The current motor speed of the wheel in rps
    */
    @Override
    public double getSpeed() {
        return m_motor.getVelocity().getValue();
    }
    
    /**
    * <h3>getVoltage</h3>
    * @return The current voltage of the motor
    */
    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValue();
    }


    /**
    * <h3>getTargetVelocity</h3>
    * @return The current voltage of the right motor
    */
    @Override
    public double getTargetVelocity() {
        return ((MotionMagicVelocityTorqueCurrentFOC) m_motor.getAppliedControl()).Velocity;
    }

    @Override
    public TalonFX getTalon() {
        return m_motor;
    }
    
    /**
    * <h3>stop</h3>
    * This sets the shooter's speed to 0
    */
    @Override
    public void stop() {
        setSpeed(0,0);
    }

    @Override
    public void runSim() {}
}
