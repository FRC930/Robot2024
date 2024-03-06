package frc.robot.subsystems.timeofflight;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.IOs.TimeOfFlightIO;
import frc.robot.utilities.SpeakerScoreUtility;

/**
 * <h3>TimeOfFlightIORobot</h3>
 * Representation of a time of flight sensor
 */
public class TimeOfFlightIORobot implements TimeOfFlightIO {

    private TimeOfFlight m_sensor;
    private double m_dist;

    public TimeOfFlightIORobot(int id, double triggerDistance) {
        m_sensor = new TimeOfFlight(id);
        m_dist = triggerDistance;
        m_sensor.setRangingMode(RangingMode.Short, 25);
    }

    public boolean get() {
        return m_sensor.getRange() < m_dist;
        //Logger.recordOutput(TimeOfFlightIORobot.class.getSimpleName() + "/frequency", );
    }
    
    public double getRange() {
        return m_sensor.getRange();
    }

    // public void setRefreshRate(double refreshSeconds) {
    //     m_sensor.setRangingMode(RangingMode.Short, refreshSeconds);
    // }
}