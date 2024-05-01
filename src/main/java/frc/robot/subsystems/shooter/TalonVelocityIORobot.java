package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.IOs.TalonVelocityIO;
import frc.robot.utilities.Phoenix6Utility;

public class TalonVelocityIORobot implements TalonVelocityIO{
    private double gearRatio;

    protected TalonFX m_motor; 

    protected VelocityTorqueCurrentFOC m_request;

    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();


    protected TalonVelocityIORobot(int MotorID, double gearRatio,Slot0Configs config,Slot1Configs config1, MotionMagicConfigs mmConfigs, boolean initReal, MotionMagicVelocityVoltage simRequest) {
        m_motor = new TalonFX(MotorID); //Initializes the motor

        this.gearRatio = gearRatio; // The gear ratio
          
        m_request = new VelocityTorqueCurrentFOC(0.0,0.0,0.0,0,true,false,false); //The request that will be sent to the motor

        TalonFXConfiguration cfg = new TalonFXConfiguration(); //Creates a new blank TalonFX congfiguration that will be applied to the motors in a bit
        cfg.withSlot0(config); // The PID and FF configs
        cfg.withSlot1(config1);
        cfg.Feedback.SensorToMechanismRatio = this.gearRatio;
        cfg.withMotionMagic(mmConfigs); // The Motion Magic configs

        // cfg.Voltage.PeakForwardVoltage = 8;
        // cfg.Voltage.PeakReverseVoltage = -8;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 150;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -150;

        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 80.0;

        Phoenix6Utility.setTalonFxConfiguration(m_motor, cfg); //Applies the configuration to the motor

        m_motor.setNeutralMode(NeutralModeValue.Coast); //Makes the motor continue rotating even when it is told to brake (its velocity is set to 0)

        Phoenix6Utility.applyConfigAndRetry(m_motor, () -> m_motor.setControl(new NeutralOut()));
        
    }

    public TalonVelocityIORobot(int MotorID, double gearRatio,Slot0Configs config, Slot1Configs config1, MotionMagicConfigs mmConfigs) {
        this(MotorID, gearRatio,config, config1,  mmConfigs, true, null);
    }



    /**
    * <h3>setSpeed</h3>
    * @param speed the speed the wheel will be set to
    */
    @Override
    public void setSpeedWithSlot(double speed, double acceleration, int slot) {
        ControlRequest request;
        if(speed == 0.0) {
            request = m_brake;
        } else {
            request = m_request.withVelocity(speed).withAcceleration(acceleration).withSlot(slot);
        }
        Phoenix6Utility.applyConfigAndNoRetry(
            m_motor,
            () -> m_motor.setControl(request));
    }

    public void setSpeed(double speed, double acceleration) {
        setSpeedWithSlot(speed, acceleration, 0);
    }

    /**
    * <h3>setSpeed</h3>
    * @param speed the speed in the wheel will be set to in rot/s
    */
    @Override
    public void setSpeedWithSlot(double speed, int slot) {
        ControlRequest request;
        if(speed == 0.0) {
            request = m_brake;
        } else {
            request = m_request.withVelocity(speed).withSlot(slot);
        }
        Phoenix6Utility.applyConfigAndNoRetry(
            m_motor,
            () -> m_motor.setControl(request));
    }

    public void setSpeed(double speed) {
        setSpeedWithSlot(speed, 0);
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
        return Phoenix6Utility.getVelocityFromController(m_motor, 0.0);
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

    /**
     * Sets slot 0
     */
    public void setSlot(Slot0Configs config) {
        m_motor.getConfigurator().apply(config);
    }

    /**
     * Sets slot 1
     */
    public void setSlot(Slot1Configs config) {
        m_motor.getConfigurator().apply(config);
    }

    /**
     * Sets slot 2
     */
    public void setSlot(Slot2Configs config) {
        m_motor.getConfigurator().apply(config);
    }

    public TalonVelocityIORobot withSlot(Slot0Configs config) {
        setSlot(config);
        return this;
    }

    public TalonVelocityIORobot withSlot(Slot1Configs config) {
        setSlot(config);
        return this;
    }

    public TalonVelocityIORobot withSlot(Slot2Configs config) {
        setSlot(config);
        return this;
    }
}
