package frc.robot.subsystems.timeofflight;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.IOs.TimeOfFlightIO;

public class TimeOfFlightIORobot implements TimeOfFlightIO {

    private TimeOfFlight m_sensor;
    private double m_dist;

    public TimeOfFlightIORobot(int id, double triggerDistance) {
        m_sensor = new TimeOfFlight(id);
        m_dist = triggerDistance;
    }

    public boolean get() {
        return m_sensor.getRange() < m_dist;
    }
}