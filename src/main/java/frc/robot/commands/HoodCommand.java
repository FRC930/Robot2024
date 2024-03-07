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
    private double m_startTime;
    private DIOSim m_dio; 
    private final double m_speed;
    
    private static double CURRENT_THRESHOLD = 4.0;
    private static double CHECK_DELAY = 0.2;

    public HoodCommand(AmpHoodSubsystem hood, double speed) {
        m_hood = hood;
        m_speed = speed;
        m_startTime = Timer.getFPGATimestamp();
        DIOSim simdio = new DIOSim(5);
        simdio.setIsInput(true);
        simdio.setInitialized(true);
        simdio.setValue(false);
        m_dio = simdio;
    }

    @Override
    public void initialize() {
        m_hood.setSpeed(m_speed);
        m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        if(Robot.isReal()) {
            return m_hood.getCurrent() > CURRENT_THRESHOLD && (Timer.getFPGATimestamp() - m_startTime) > CHECK_DELAY;
        } else {
            return m_dio.getValue();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.setSpeed(0);
    }
}
