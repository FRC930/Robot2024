package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.commands.HoodCommand;
import frc.robot.utilities.Phoenix6Utility;

@Deprecated
public class AmpHoodSubsystem extends SubsystemBase{
    private TalonRollerIO m_io;
    private static final double extendedAngleThreshold = 9999; //TODO set both
    private static final double retractedAngleThreshold = -9999;

    private static final double HOOD_IN_SPEED = -0.1;
    private static final double HOOD_OUT_SPEED = 0.1;

    public AmpHoodSubsystem(TalonRollerIO io) {
        m_io = io;

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 5.0;
        cfg.Voltage.PeakForwardVoltage = 6.0;
        cfg.Voltage.PeakReverseVoltage = 6.0;
        
        m_io.getTalon().setNeutralMode(NeutralModeValue.Brake);
        
        Phoenix6Utility.applyConfigAndRetry(m_io.getTalon(), ()-> m_io.getTalon().getConfigurator().apply(cfg));
        
    }

    /**
    * <h3>setSpeed</h3>
    * @param speed the speed the wheel will be set to [-1-1]
    */
    public void setSpeed(double speed) {
        m_io.setSpeed(MathUtil.clamp(speed, -1.0, 1.0));
    }

    /**
    * <h3>geMotorSpeed</h3>
    * @return The current motor speed of the wheel in rps
    */
    public double getSpeed() {
        return m_io.getSpeed();
    }
    
    /**
    * <h3>getVoltage</h3>
    * @return The current voltage of the motor
    */
    public double getVoltage() {
        return m_io.getVoltage();
    }

    /**
    * <h3>getCurrent</h3>
    * @return The current voltage of the motor
    */
    public double getCurrent() {
        return m_io.getStatorCurrent();
    }
    
    /**
    * <h3>stop</h3>
    * This sets the shooter's speed to 0
    */
    public void stop() {
        setSpeed(0.0);
    }

    public boolean getIsExtended() {
        System.out.print("");
        return m_io.getTalon().getPosition().getValue() > extendedAngleThreshold;
    }

    public boolean getIsRetracted() {
        System.out.print("");
        return m_io.getTalon().getPosition().getValue() < retractedAngleThreshold;
    }

    public boolean getIsInMotion() {
        return !(getIsExtended() || getIsRetracted());
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.getClass().getSimpleName() + "/Speed", getSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/PositionRots", m_io.getTalon().getPosition().getValue());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage", getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Current", getCurrent());
    }

    public Command newWaitUntilAmpIsExtendedCommand() {
        //return new WaitUntilCommand(this::getIsExtended);
        return new InstantCommand();

    }

    public Command newWaitUntilAmpIsRetractedComand() {
        //return new WaitUntilCommand(this::getIsExtended);
        return new InstantCommand();
    }

    public Command newExtendHoodCommand() {
        //return new HoodCommand(this, HOOD_OUT_SPEED);
        return new InstantCommand();
    }

    public Command newRetractHoodCommand() {
        //return new HoodCommand(this, HOOD_IN_SPEED);
        return new InstantCommand();
    }
}
