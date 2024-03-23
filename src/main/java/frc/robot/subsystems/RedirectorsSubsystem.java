package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;

public class RedirectorsSubsystem extends SubsystemBase{
    private static final double RedirectorExtendVoltage = 0.5;
    private static final double RedirectorRetractVoltage = -0.5;

    private static final int MotorFreeLimit = 30; // Amps
    private static final int MotorStallLimit = 10; // Amps

    private SparkMaxWrapper m_motor;

    public RedirectorsSubsystem(int deviceID) {
        m_motor = new SparkMaxWrapper(deviceID, MotorType.kBrushless);

        m_motor.setSmartCurrentLimit(MotorFreeLimit, MotorStallLimit);

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
        }
        // Sets motor so it can't be manually moved when neutral
        m_motor.setIdleMode(IdleMode.kBrake);

        // Motor is not inverted
        m_motor.setInverted(false);
    }

    /**
     * <h3> extendIntake</h3>
     * Tells the redirector to extend/retract depending on the voltage applied.
     */
    public void setVoltage(double voltage) {
         // sets the voltage boundries
         m_motor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public void simulationPeriodic() {
      REVPhysicsSim.getInstance().run();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/OutputAmps", m_motor.getOutputCurrent());
        Logger.recordOutput(this.getClass().getSimpleName()+"/Position", m_motor.getIOPosition());
    }

    public Command getNewExtendCommand() {
        return new InstantCommand(()->setVoltage(RedirectorExtendVoltage));
    }

    public Command getNewRetractCommand() {
        return new InstantCommand(()->setVoltage(RedirectorRetractVoltage));
    }

    
}
