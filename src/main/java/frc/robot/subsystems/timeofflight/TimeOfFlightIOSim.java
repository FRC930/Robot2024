package frc.robot.subsystems.timeofflight;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.IOs.TimeOfFlightIO;

/**
 * <h3>TimeOfFlightIOSim</h3>
 * Simulated representation of a time of flight sensor
 */
public class TimeOfFlightIOSim implements TimeOfFlightIO {

    private DigitalInput m_sensor;

    public TimeOfFlightIOSim(int id) {
        m_sensor = new DigitalInput(id);
    }

    public boolean get() {
        return m_sensor.get();
    }

    @Override
    public double getRange() {
        return -1;
    }
}