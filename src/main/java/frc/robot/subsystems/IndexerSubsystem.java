package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
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
    private Notifier m_sensorNotifier;

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
            return new WaitCommand(.1);
        }
    }

    public Command newUnlessNoteFoundCommand() {
        if(Robot.isReal()) {
            return new WaitUntilCommand(() -> !getSensor());  // DO not set subsystem since just getting sensor value
        } else {
            return new WaitCommand(.1);
        }
    }

    public void stopOnNextNoteDetected() {
        Logger.recordOutput(this.getClass().getSimpleName() + "/WaitingForNote",true);
        m_sensorNotifier.setCallback(this::StopIfNoteDetectedCallback);
        m_sensorNotifier.startPeriodic(M_HF_SENSOR_PERIOD);
    }

    private void StopIfNoteDetectedCallback() {
        if(getSensor()) {
            Logger.recordOutput(this.getClass().getSimpleName() + "/WaitingForNote",false);
            stop();
            this.m_sensorNotifier.stop();
            // TODO USE THIS IF WE HAVE ISSUES WITH THE INDEXER NOT STOPPING 
            // This is sketchy, but I don't want the command to run another time and make the indexer keep going. 
            // I am more afraid of it becoming a time wasting ghost error though, so I am leaving it out. 

            //getCurrentCommand().cancel();
            
            return;
        }
    }

    

}
