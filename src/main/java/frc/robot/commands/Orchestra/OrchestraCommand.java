package frc.robot.commands.Orchestra;

import java.util.ArrayList;
import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class OrchestraCommand extends Command {
    private Orchestra m_orchestra;
    private ArrayList<TalonFX> m_instruments;
    private String m_file;

    private static final String LOGGER_SONGTIME_PATH = "Orchestra/SongTime";
    private static final String LOGGER_SONGPLAYING_PATH = "Orchestra/SongPlaying";
    private static final String LOGGER_SONGSTATE_PATH = "Orchestra/SongState";
    
    public OrchestraCommand(Orchestratable... Sections) {
        m_instruments = new ArrayList<TalonFX>();
        m_orchestra = new Orchestra();
        for(Orchestratable o : Sections) {
            m_instruments.addAll(Arrays.asList(o.getInstruments()));
            addRequirements(o.getSubsystems());
        }
        Logger.recordOutput(LOGGER_SONGTIME_PATH,0.0);
        Logger.recordOutput(LOGGER_SONGPLAYING_PATH,false);
        Logger.recordOutput(LOGGER_SONGSTATE_PATH,"Not Started");
    }

    public OrchestraCommand(String song, Orchestratable... Sections) {
        this(Sections);
        setSong(song);
    }

    public void setSong(String filepath) {
        m_file = filepath;
    }

    @Override
    public void initialize() {
        if(m_file == null) {
            System.err.println("No Orchestra song set!");
            return;
        }
        StatusCode result = m_orchestra.loadMusic(m_file);
        if(!result.isOK()) {
            System.err.println("Orchestra returned bad status code while loading music! Error:");
            System.err.println(result);
            return;
        }
        for(TalonFX talon : m_instruments) {
            StatusCode code = m_orchestra.addInstrument(talon);
            if(!code.isOK()) {
                System.err.println("Orchestra returned bad status code while loading instrument [%s]! Error:".formatted(talon));
                System.err.println(code);
            }
        }
        m_orchestra.play();

        Logger.recordOutput(LOGGER_SONGTIME_PATH,0.0);
        Logger.recordOutput(LOGGER_SONGPLAYING_PATH,true);
        Logger.recordOutput(LOGGER_SONGSTATE_PATH,"Playing");
    }
    
    @Override
    public void execute() {
        Logger.recordOutput(LOGGER_SONGTIME_PATH,m_orchestra.getCurrentTime());
    }

    @Override
    public boolean isFinished() {
        return false;
        //return !m_orchestra.isPlaying();
    }

    @Override
    public void end(boolean interrupted) {
        m_orchestra.stop();
        Logger.recordOutput(LOGGER_SONGTIME_PATH,0.0);
        Logger.recordOutput(LOGGER_SONGPLAYING_PATH,false);
        Logger.recordOutput(LOGGER_SONGSTATE_PATH,interrupted?"Interrupted":"Ended");
    }
}
