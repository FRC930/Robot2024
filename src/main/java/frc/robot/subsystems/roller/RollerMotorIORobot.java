package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.IOs.TalonRollerIO;
import frc.robot.utilities.Phoenix6Utility;

/**
 * <h3>RollerMotorIORobot</h3>
 * Representation of a roller motor on the robot
 */
public class RollerMotorIORobot implements TalonRollerIO {

    protected TalonFX m_motor; // Protected because needed by IOSim

    public RollerMotorIORobot(int id, String canbus) {
        m_motor = new TalonFX(id, canbus);
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        //cfg.CurrentLimits.SupplyCurrentLimitEnable = true; 
        // cfg.CurrentLimits.SupplyCurrentThreshold = 0; // the peak supply current, in amps 
        // cfg.CurrentLimits.SupplyTimeThreshold = 1.5; // the time at the peak supply current before the limit triggers, in sec
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 150.0;

        Phoenix6Utility.setTalonFxConfiguration(m_motor, cfg);

    }

    public RollerMotorIORobot(int id, String canbus, double statorCurrentLimit, double supplyCurrentLimit) {
        this(id, canbus);
        //Setting current limits
        m_motor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(statorCurrentLimit).withSupplyCurrentLimit(supplyCurrentLimit));
    } 

    // public void applyConfigsToMotor(TalonFXConfiguration tfxConfig) {
    //     m_motor.getConfigurator().apply(tfxConfig);
    // }

    @Override
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    @Override
    public double getSpeed() {
        return m_motor.getVelocity().getValue();
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValue();
    }

    @Override
    public void runSim() {}

    @Override
    public TalonFX getTalon() {
        return m_motor;
    }

    @Override
    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValue();
    }

    @Override
    public double getInputCurrent() {
        return m_motor.getSupplyCurrent().getValue();
    }
}