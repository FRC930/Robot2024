package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.IOs.TalonPosIO;
import frc.robot.IOs.TimeOfFlightIO;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.Phoenix6Utility;
import frc.robot.utilities.SpeakerScoreUtility;

/**
 * <h3>PivotSubsystem</h3>
 * A subsystem that represents the pivot
 */
public class PivotSubsystem extends SubsystemBase{

    private static final boolean ENABLE_REZEROING = true;

    private final TalonPosIO m_io;
    private TimeOfFlightIO m_sensorIO;

    private boolean m_reachedSetPoint = false;

    private boolean m_resetWhenSensorTriggered = false;
    private boolean m_allowToBeReset = false;

    Debouncer m_debouncer = new Debouncer(1.0, Debouncer.DebounceType.kRising);


    public static final double PIVOT_DEADBAND = 0.5;

    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public PivotSubsystem(TalonPosIO io, TimeOfFlightIO ToF) {
        m_io = io;
        m_sensorIO = ToF;
        //Setting stow pos on robot startup
        setPosition(CommandFactoryUtility.PIVOT_STOW_POS);
    }

    /**
     * <h3>setPosition</h3>
     * Sets the target angle of the subsystem
     * @param angle The angle in degrees from the horizontal
     */
    public void setPosition(double angle) {
        m_io.setTarget(MathUtil.clamp(angle,0.0,90.0));        
        m_reachedSetPoint = false;
        Logger.recordOutput(this.getClass().getSimpleName() + "/ReachedSetPoint", m_reachedSetPoint);
        if (angle == 0.0){
            m_resetWhenSensorTriggered = true;
        }
    }

    public void resetMotorPosition() {
        // reset motor
        if(m_io instanceof PivotIORobot && ENABLE_REZEROING) {
            Logger.recordOutput(this.getClass().getSimpleName() + "/AngleWhenReset", getPosition());
            Logger.recordOutput(this.getClass().getSimpleName() + "/LimitSwitchRangeWhenReset", m_sensorIO.getRange());
            TalonFX m_motor = ((PivotIORobot) m_io).m_motor;    
            Phoenix6Utility.applyConfigAndNoRetry(m_motor,
                () -> m_motor.getConfigurator().setPosition(Units.degreesToRotations(1.67))); // It boots as 1.67 instead of 0.0
        }
    }

    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getTarget() {
        return m_io.getTarget();
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from the horizontal
     */
    public double getPosition() {
        return m_io.getPos();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the velocity in degrees per second where 0 is the horizontal and positive is up.
     * @return
     */
    public double getVelocity() {
        return m_io.getVelocity();
    }

    /**
     * <h3>getVoltage</h3>
     * Gets the current voltage of the subystem.
     * @return The voltage the motor is running at.
     */
   // public double getVoltage() {

   // }


    /**
     * <h3>getSensor</h3>
     * @return value of indexer sensor
     */
    public boolean getSensor() {
        return m_sensorIO.get();
    }

    @Override
    public void periodic() {
        m_io.runSim();
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity", getVelocity());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Angle", getPosition());
        Logger.recordOutput(this.getClass().getSimpleName() + "/SetPoint", getTarget());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage", m_io.getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LimitSwitch", getSensor());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LimitSwitchRange", m_sensorIO.getRange());
        if(m_allowToBeReset && m_resetWhenSensorTriggered){
            if(m_debouncer.calculate(getSensor())){ //TODO: Debounce value
                m_allowToBeReset = false;
                m_resetWhenSensorTriggered = false;
                resetMotorPosition();
            }
        }
        
    }

    public Command newSetPosCommand(double pos) {
        return new InstantCommand(() -> setPosition(pos), this);
    }

    public Command newSetPosCommand(SpeakerScoreUtility speakerUtil) {
        return new InstantCommand(() -> setPosition(speakerUtil.getPivotAngle()), this);
    }

    public Command newCalcAndSetPosCommand() {
        return new InstantCommand(
            () -> setPosition(SpeakerScoreUtility.computePivotAngle(SpeakerScoreUtility.inchesToSpeaker())), this
        );
    }

    public boolean atSetpoint() {
        double pos = getPosition(); 
        double target = getTarget(); 
        m_reachedSetPoint = MathUtil.applyDeadband(target - pos, PIVOT_DEADBAND) == 0.0;
        if (target > 0.0 && m_reachedSetPoint) { //TODO: doesn't work if atSetpoint never finishes
            m_allowToBeReset = true;
        }
        Logger.recordOutput(this.getClass().getSimpleName() + "/ReachedSetPoint", m_reachedSetPoint);
        return m_reachedSetPoint;
    }

    public Command newWaitUntilSetpointCommand(double timeout) {
        return new WaitCommand(timeout).until(() -> atSetpoint()); // Not dependent on subsystem because can run parralel with set position
    }
}
