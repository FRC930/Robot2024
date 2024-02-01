package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerEncoderIO;

public class TurretSubsystem extends SubsystemBase{
    private final TalonRollerEncoderIO m_io;

    private final String turretName;

    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public TurretSubsystem(TalonRollerEncoderIO io) {
        m_io = io;
        turretName = "" + this.hashCode();
    }

    public void setSpeed(double speed) {
        m_io.setSpeed(speed);
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from 0
     */
    public double getPosition() {
        return m_io.getDegrees();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the velocity in degrees per second where 0 is the horizontal and positive is up.
     * @return
     */
    public double getVelocity() {
        return m_io.getSpeed();
    }

    public double getVoltage() {
        return m_io.getVoltage();
    }


    @Override
    public void periodic() {
        m_io.runSim();
        SmartDashboard.putNumber("TurretVoltage-" + turretName, getVoltage());
        SmartDashboard.putNumber("TurretVelocity-" + turretName,getVelocity());
        SmartDashboard.putNumber("TurretDegrees-" + turretName,getPosition());
        SmartDashboard.putNumber("TurretRotations-" + turretName,m_io.getMechRotations());
    }

}
