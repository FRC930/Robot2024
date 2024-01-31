package frc.robot.subsystems.timeofflight;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.IOs.TimeOfFlightIO;

public class TimeOfFlightIOSim implements TimeOfFlightIO {

    private DigitalInput m_sensor;

    public TimeOfFlightIOSim(int id) {
        m_sensor = new DigitalInput(id);
    }

    public boolean get() {
        return m_sensor.get();
    }
}