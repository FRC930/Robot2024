package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class PivotSubsystem extends SubsystemBase{
    private final PivotIO io;

    private ProfiledPIDController pidController;
    private ArmFeedforward ffController;

    private double targetAngle;

    private final double MAX_VELOCITY = 100;
    private final double MAX_ACCELERATION = 120;

    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public PivotSubsystem(int motorID) {
        pidController = new ProfiledPIDController(1, 0, 0, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        ffController = new ArmFeedforward(0, 1, 0);

        this.io = RobotBase.isReal() ? new PivotIORobot(motorID): new PivotIOSim();
        targetAngle = 0;
    }

    /**
     * <h3>setPosition</h3>
     * Sets the target angle of the subsystem
     * @param angle The angle in degrees from the horizontal
     */
    public void setPosition(double angle) {
        targetAngle = MathUtil.clamp(angle,0,180);
    }

    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getSetPoint() {
        return targetAngle;
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from the horizontal
     */
    public double getPosition() {
        return io.getCurrentAngleDegrees();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the velocity in degrees per second where 0 is the horizontal and positive is up.
     * @return
     */
    public double getVelocity() {
        return io.getVelocityDegreesPerSecond();
    }


    @Override
    public void periodic() {
        // Always runs if the robot is in sim, only runs IRL if robot is enabled.
        if(DriverStation.isEnabled() || !Robot.isReal()) {
            double effort = pidController.calculate(getPosition(),getSetPoint());
            double feedforward = ffController.calculate(Units.degreesToRadians(getPosition()), Units.degreesToRadians(getVelocity()));

            effort += feedforward;
            effort = MathUtil.clamp(effort,-3,3); //TODO: Update max voltages

            io.setVoltage(effort);
        } else {
            pidController.reset(io.getCurrentAngleDegrees());
        }
    }
}
