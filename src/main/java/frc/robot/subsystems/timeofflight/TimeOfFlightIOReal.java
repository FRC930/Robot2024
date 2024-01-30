package frc.robot.subsystems.timeofflight;

import com.playingwithfusion.TimeOfFlight;

public class TimeOfFlightIOReal implements TimeOfFlightIO {

    private TimeOfFlight m_sensor;
    private double m_dist;

    public TimeOfFlightIOReal(int id, double triggerDistance) {
        m_sensor = new TimeOfFlight(id);
        m_dist = triggerDistance;
    }

    public boolean get() {
        return m_sensor.getRange() < m_dist;
    }
}