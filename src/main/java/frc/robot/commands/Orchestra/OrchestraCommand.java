package frc.robot.commands.Orchestra;

import java.util.ArrayList;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class OrchestraCommand extends Command{
    private String m_name;
    private Orchestra m_orchestra;

    private int cyclesNotPlaying = 0;

    public OrchestraCommand(String name,String musicPath,PlaysMusic... participants) {
        ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
        ArrayList<Integer> instrumentIDs = new ArrayList<Integer>();
        ArrayList<StatusCode> instrumentStatusCodes = new ArrayList<StatusCode>();
        m_orchestra = new Orchestra();
        m_name = name;

        //For each participant, load all subsystems as requirements and load all instruments
        for(PlaysMusic p : participants) {
            for(Subsystem ps : p.getSubsystems()) {
                this.addRequirements(ps);
            }
            for(TalonFX pt : p.getInstruments()) {
                instruments.add(pt);
                instrumentIDs.add(pt.getDeviceID());
                instrumentStatusCodes.add(m_orchestra.addInstrument(pt));
            }
        }
        
        //Load music
        StatusCode finalCode = m_orchestra.loadMusic(musicPath);

        //Record the initial state of the orchestra for debug
        // Logger.recordOutput("Orchestra/"+m_name+"/InitState","[MusicStatus=%s,Participants=%s,InstrumentIDs=%s,InstrumentStatuses=%s]".formatted(
        //     finalCode.toString(),
        //     participants.length,
        //     instrumentIDs.toString(),
        //     instrumentStatusCodes.toString()
        // ));
        // printHere("LatestOutput","Initialized!");
    }

    // /**
    //  * Method to make "console-like" logging easier
    //  * @param key 
    //  * @param message
    //  */
    // private void printHere(String key,String message) {
    //     Logger.recordOutput("Orchestra/"+m_name+"/"+key,Timer.getFPGATimestamp() + " - " + message);
    // }

    @Override
    public void initialize() {
        // printHere("LatestOutput",m_orchestra.play().getName());
        m_orchestra.play();
    }

    @Override
    public void execute() {
        double time = m_orchestra.getCurrentTime();
        boolean playing = m_orchestra.isPlaying();

        // Logger.recordOutput("Orchestra/"+m_name+"/currentTime",time);
        // Logger.recordOutput("Orchestra/"+m_name+"/isPlaying",playing);

        //Helps us know when we should end the orchestra.
        if(!playing && time == 0.0) {
            cyclesNotPlaying++;
        } else {
            cyclesNotPlaying = 0;
        }
    }

    @Override
    public boolean isFinished() {
        // Checks whether the orchestra has finished playing by checking if it has not played for a certain amount of loops.
        return cyclesNotPlaying > 4;
    }

    @Override
    public void end(boolean interrupted) {
        // printHere("LatestOutput",m_orchestra.stop().getName());
        m_orchestra.stop();
        cyclesNotPlaying = 0;
    }
}
