package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PosSubsystemIO;

public class TurretSubsystem extends SubsystemBase{
    private final PosSubsystemIO io;

    private final String turretName;

    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public TurretSubsystem(PosSubsystemIO io) {
        this.io = io;
        turretName = "" + this.hashCode();
    }

    /**
     * <h3>setPosition</h3>
     * Sets the target angle of the subsystem
     * @param angle The angle in degrees from the horizontal
     */
    public void setPosition(double angle) {
        io.setTarget(angle);
        
    }

    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getSetPoint() {
        return io.getTarget();
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from the horizontal
     */
    public double getPosition() {
        return io.getPos();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the velocity in degrees per second where 0 is the horizontal and positive is up.
     * @return
     */
    public double getVelocity() {
        return io.getVelocity();
    }


    @Override
    public void periodic() {
        io.runSim();
        SmartDashboard.putNumber("TurretVelocity-" + turretName,getVelocity());
        SmartDashboard.putNumber("TurretAngle-" + turretName,getPosition());
        SmartDashboard.putNumber("TurretSetpoint-" + turretName,getSetPoint());
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(()->{setPosition(90); System.out.println("Turret Test Start");}, ()->{setPosition(0);}, this);
    }

}
