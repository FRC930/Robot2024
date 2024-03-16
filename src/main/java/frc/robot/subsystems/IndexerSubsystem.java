package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;
import frc.robot.utilities.LimelightHelpers;

/**
 * <h3>IndexerSubsystem</h3>
 * This subsystem controls the indexer
 */
public class IndexerSubsystem extends SubsystemBase {
    private TalonRollerIO m_rollerIO;
    private TimeOfFlightIO m_sensorIO;
    private boolean m_sensorStatus;
    private boolean m_turnedOn = false;
    private int m_counter=0;
    private TalonRollerIO m_rollerTopIO;

    /**
     * <h3>IndexerSubsystem</h3>
     * Contains the indexer motor and time of flight through IOs, 
     * allowing it to take a physical motor or sim representation
     * @param starIndexerMotor RollerMotorIORobot (Physical) or RollerMotorIOSim (Simulation) for indexer motor
     * @param topIndexerMotor RollerMotorIORobot (Physical) or RollerMotorIOSim (Simulation) for indexer motor
     * @param ToF TimeOfFlightIORobot (Physical) or TimeOfFlightIOSim (Simulation) for indexer sensor
     */
    public IndexerSubsystem(TalonRollerIO starIndexerMotor, TalonRollerIO topIndexerMotor, TimeOfFlightIO ToF) {
        m_rollerIO = starIndexerMotor;
        m_rollerTopIO = topIndexerMotor;
        m_sensorIO = ToF;

        starIndexerMotor.getTalon().setInverted(false);
        starIndexerMotor.getTalon().setNeutralMode(NeutralModeValue.Coast); // Applies brake mode to belt
        topIndexerMotor.getTalon().setInverted(true);
        topIndexerMotor.getTalon().setNeutralMode(NeutralModeValue.Brake); // Applies brake mode to belt

    }


    /**
     * <h3>setIndexerSpeed</h3>
     * @param speed PercentOutput speed to apply
     */
    public void setStarIndexerSpeed(double speed) {
        m_rollerIO.setSpeed(speed / 2.0); // TODO take out once pulley installed
    }

    public void setStarIndexerVoltage(double voltage) {
        m_rollerIO.getTalon().setVoltage(voltage);
    }

    /**
     * <h3>setAmpSpeed</h3>
     * @param speed PercentOutput speed to apply
     */
    public void setTopIndexerSpeed(double speed) {
        m_rollerTopIO.setSpeed(speed);
    }

     public void setTopIndexerVoltage(double voltage) {
        m_rollerTopIO.getTalon().setVoltage(voltage);
    }

    /**
     * <h3>setSpeed</h3>
     * @param speed PercentOutput speed to apply
     */
    public void setSpeed(double speed) {
        setStarIndexerSpeed(speed);  
        setTopIndexerSpeed(speed);
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
        return m_sensorStatus;
    }

    public double getSensorDistance() {
        return m_sensorIO.getRange();
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(() -> {setSpeed(0.3);}, () -> {stop();}, this);
    }

    @Override
    public void periodic() {
        m_sensorStatus = m_sensorIO.get();
        if(m_sensorStatus) {
            if(!m_turnedOn) {
                LimelightHelpers.setLEDMode_ForceBlink("limelight-front"); 
                LimelightHelpers.setLEDMode_ForceBlink("limelight-back"); 
                m_turnedOn = true;
            } else {
                m_counter++;
                if(m_counter>10) {
                    LimelightHelpers.setLEDMode_ForceOff("limelight-front"); 
                    LimelightHelpers.setLEDMode_ForceOff("limelight-back"); 
                }
            }
        } else {
            if(m_turnedOn) {
                LimelightHelpers.setLEDMode_ForceOff("limelight-front"); 
                LimelightHelpers.setLEDMode_ForceOff("limelight-back"); 
                m_turnedOn = false;
                m_counter = 0;
            }
        }
        Logger.recordOutput(this.getClass().getSimpleName() + "/Star/Velocity" ,getSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Star/Voltage" ,getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Top/Velocity" ,m_rollerTopIO.getSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Top/Voltage" ,m_rollerTopIO.getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Sensor", getSensor());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LastSensorCheck", Timer.getFPGATimestamp());
        Logger.recordOutput(this.getClass().getSimpleName() + "/SensorRange", m_sensorIO.getRange());
    }

    public Command newSetSpeedCommand(double speed) {
        return new InstantCommand(() -> setSpeed(speed), this);
    }

    public Command newSetStarSpeedCommand(double speed) {
        return new InstantCommand(() -> setStarIndexerSpeed(speed), this);
    }

    public Command newSetTopSpeedCommand(double speed) {
        return new InstantCommand(() -> setTopIndexerSpeed(speed), this);
    }

    public Command newSetStarVoltageCommand(double speed) {
        return new InstantCommand(() -> setStarIndexerVoltage(speed), this);
    }

    public Command newSetTopVoltageCommand(double speed) {
        return new InstantCommand(() -> setTopIndexerVoltage(speed), this);
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


}
