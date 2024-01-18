package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO IO;

    private PIDController pidController;
    private ElevatorFeedforward ffController;
    private ElevatorFeedforward topffController;

    private double targetHeight;

    //TODO: This is copied from last year's code because it seemed important, and I need to review what this subsystem is actually doing.
    private static final double UPPER_STAGE_HIGHT = 20.0; 

    /**
     * <h3>ElevatorSubsystem</h3>
     * Creates a subsystem representing the elevator on the robot.
     * @return
     */
    public ElevatorSubsystem(ElevatorIO io) {
        this.IO = io;
        this.pidController = new PIDController(30, 0, 0.3);
        this.ffController = new ElevatorFeedforward(0.0, 0.35, 0.0, 0.0);
        this.topffController = new ElevatorFeedforward(0, 0.35, 0.0, 0.0);
    
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

            double feedforward;
            if(getHeight() < Units.inchesToMeters(UPPER_STAGE_HIGHT)) {
                feedforward  = ffController.calculate(IO.getCurrentVelocity());
            }else {
                feedforward  = topffController.calculate(IO.getCurrentVelocity());
            }

            effort += feedforward;
            //Kraken max voltage: 24V
            effort = MathUtil.clamp(effort,-24,24); //TODO: Check how to make max voltages a constant

            IO.setVoltage(effort);
        }
    }


}
