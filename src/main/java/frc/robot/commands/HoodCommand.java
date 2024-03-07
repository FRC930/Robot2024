package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.AmpHoodSubsystem;


public class HoodCommand extends Command {
    private AmpHoodSubsystem m_hood;
    private HoodSetting m_setting;
    private double m_startTime;
    private DIOSim m_DIO; 
    
    private static double CURRENT_THRESHOLD = 4.0;
    private static double START_CHECK_TIME = 1.0;

    public enum HoodSetting {
        EXTEND,
        RETRACT;
    }

    public HoodCommand(AmpHoodSubsystem hood, HoodSetting setting) {
        m_hood = hood;
        m_setting = setting;
        m_startTime = Timer.getFPGATimestamp();
        DIOSim simdio = new DIOSim(5);
        simdio.setIsInput(true);
        simdio.setInitialized(true);
        simdio.setValue(false);
        m_DIO = simdio;
    }

    @Override
    public void initialize() {
        m_hood.setSpeed(m_setting == HoodSetting.EXTEND ? 1.0 : -1.0);
        m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        if(Robot.isReal()) {
            return m_hood.getCurrent() > CURRENT_THRESHOLD && (Timer.getFPGATimestamp() - m_startTime) > START_CHECK_TIME;
        } else {
            return m_DIO.getValue();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.setSpeed(0);
    }
}
