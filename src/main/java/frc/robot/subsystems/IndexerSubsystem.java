package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;
import edu.wpi.first.wpilibj.Notifier;

/**
 * <h3>IndexerSubsystem</h3>
 * This subsystem controls the indexer
 */
public class IndexerSubsystem extends SubsystemBase {
    private static final double M_HF_SENSOR_PERIOD = 0.01;
    private TalonRollerIO m_rollerIO;
    private TimeOfFlightIO m_sensorIO;
    private Notifier stopIndexerNotifier;

    /**
     * <h3>IndexerSubsystem</h3>
     * Contains the indexer motor and time of flight through IOs, 
     * allowing it to take a physical motor or sim representation
     * @param motor RollerMotorIORobot (Physical) or RollerMotorIOSim (Simulation) for indexer motor
     * @param ToF TimeOfFlightIORobot (Physical) or TimeOfFlightIOSim (Simulation) for indexer sensor
     */
    public IndexerSubsystem(TalonRollerIO motor, TimeOfFlightIO ToF) {
        m_rollerIO = motor;
        m_sensorIO = ToF;

        motor.getTalon().setNeutralMode(NeutralModeValue.Brake); // Applies brake mode to belt
        m_sensorNotifier.setCallback(()->{});
        m_sensorNotifier.setName("IndexerHFSensorNotifier");
        Logger.recordOutput(this.getClass().getSimpleName() + "/WaitingForNote",false);
    }

    /**
     * <h3>setSpeed</h3>
     * @param speed PercentOutput speed to apply
     */
    public void setSpeed(double speed) {
        m_rollerIO.setSpeed(speed);
    }

    /**
     * <h3>stop</h3>
     * Sets speed of motor to 0
     */
    public void stop() {
        setSpeed(0);
    }

    /**
     * <h3>getSpeed</h3>
     * @return current velocity of indexer as reported by the Talon
     */
    public double getSpeed() {
        return m_rollerIO.getSpeed();
    }

    /**
     * <h3>getVoltage</h3>
     * @return current applied voltage to Talon
     */
    public double getVoltage() {
        return m_rollerIO.getVoltage();
    }

    /**
     * <h3>getSensor</h3>
     * @return value of indexer sensor
     */
    public boolean getSensor() {
        return m_sensorIO.get();
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(() -> {setSpeed(0.3);}, () -> {stop();}, this);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity" ,getSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage" ,getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Sensor", getSensor());
        Logger.recordOutput(this.getClass().getSimpleName() + "/SensorRange", m_sensorIO.getRange());
    }

    public Command newSetSpeedCommand(double speed) {
        return new InstantCommand(() -> setSpeed(speed), this);
    }

    public Command newUntilNoteFoundCommand() {
        if(Robot.isReal()) {
            return new WaitUntilCommand(() -> getSensor());  // DO not set subsystem since just getting sensor value
        } else {
            return new WaitCommand(.5);
        }
    }

    public Command newUntilNoNoteFoundCommand() {
        if(Robot.isReal()) {
            return new WaitUntilCommand(() -> !getSensor());  // DO not set subsystem since just getting sensor value
        } else {
            return new WaitCommand(.1);
        }
    }

    
    /** 
     * Sets the Indexer to 0 once the Indexer sensor becomes true.
     * <p> 0.001 = 1ms
    */
    public void  watchIndexerSensorAndStop(double indexerCheckPeriod) {
        if(stopIndexerNotifier != null) {
            return; // We just continue watching
            //stopWatchingIndexerSensor(); // OR we start a new notifier
        }
        stopIndexerNotifier = new Notifier(() -> {
            if(getSensor() && stopIndexerNotifier != null) {
                stop();
                stopWatchingIndexerSensor();
            }
        });
        stopIndexerNotifier.startPeriodic(0.001);
    }

    /**
     * Stops watching the Indexer sensor, but does not stop the Indexer.
     * <p> If using this in a timeout, call stop() as well.
     */
    public void stopWatchingIndexerSensor() {
        stopIndexerNotifier.stop();
        stopIndexerNotifier.close();
    }

    /**
     * Stops the indexer whenever it detects that it has a note
     * @return the command that does that 
     */
    public Command getStopIndexerWhenSensorTriggeredCommand(double indexerCheckPeriod) {
        return new StartEndCommand(
            () -> {
                watchIndexerSensorAndStop(indexerCheckPeriod);
            }, () -> {
                stop();
                stopWatchingIndexerSensor();
            }
        );
    }

    /**
     * Stops the indexer whenever it detects that it has a note.
     * <p> This one 
     * @return the command that does that 
     */
    public Command getStopIndexerWhenSensorTriggeredCommand(double indexerCheckPeriod, double timeout) {
        return getStopIndexerWhenSensorTriggeredCommand(indexerCheckPeriod).raceWith(new WaitCommand(timeout));
    }
}
