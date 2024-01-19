package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.constants.MotorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO IO;
    private final MotorConstants motorConstants;

    private PIDController pidController;
    private ElevatorFeedforward ffController;

    private double targetHeight;


    /**
     * <h3>ElevatorSubsystem</h3>
     * Creates a subsystem representing the elevator on the robot.
     * @return
     */
    public ElevatorSubsystem(ElevatorIO io,double maxHeight, String elevatorName) {
        this.IO = io;

        this.pidController = new PIDController(30, 0, 0.3);
        this.ffController = new ElevatorFeedforward(0.0, 0.35, 0.0, 0.0); //TODO: Does not work for multi-elevator but we're revamping code on saturday so that's a problem for later us
        this.motorConstants = MotorConstants.getInstance();
    
        this.pidController.setTolerance(0.5, 0.5);
    }

    /**
     * <h3>setTargetHeight</h3>
     * Sets the target height of the elevator
     * @param targetHeight The height the robot will try to move to
     */
    public void setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
    }

    /**
     * <h3>getHeight</h3>
     * Gets the current height of the elevator
     * @return The height of the elevator
     */
    public double getHeight() {
        return IO.getCurrentHeight();
    }

    /**
     * <h3>getTargetHeight</h3>
     * Gets the height the elevator is trying to move to.
     * @return The target height
     */
    public double getTargetHeight() {
        return targetHeight;
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the current velocity of the elevator
     * @return The velocity of the elevator
     */
    public double getVelocity() {
        return IO.getCurrentVelocity();
    }

    @Override
    public void periodic() {
        IO.updateInputs();
        // Always runs if the robot is in sim, only runs IRL if robot is enabled.
        if(DriverStation.isEnabled() || !Robot.isReal()) {
            double effort = pidController.calculate(getHeight(),targetHeight);
            effort += ffController.calculate(IO.getCurrentVelocity());

            effort = MathUtil.clamp(effort,-motorConstants.KRAKEN_MAX_VOLTAGE,motorConstants.KRAKEN_MAX_VOLTAGE);
            IO.setVoltage(effort);
        }
    }


}
