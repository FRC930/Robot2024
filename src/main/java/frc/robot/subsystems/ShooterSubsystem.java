package frc.robot.subsystems;

 import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.IOs.TalonVelocityIO;
import frc.robot.utilities.Phoenix6Utility;
import frc.robot.utilities.SpeakerScoreUtility;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * <h3>ShooterSubsystem</h3>
 * This subsystem controls the shooter
 */
public class ShooterSubsystem extends SubsystemBase{

    private TalonVelocityIO IO_Left;
    private TalonVelocityIO IO_Right;

    private final double VELOCITY_DEADBAND = 5.0;
    private boolean m_reachedSetPoint = false;

    public ShooterSubsystem(TalonVelocityIO LeftIO, TalonVelocityIO RightIO) { 
        // m_leftMotor = new TalonFX(leftID);
        IO_Left = LeftIO;
        IO_Right = RightIO;

        IO_Left.getTalon().setInverted(true);
        IO_Right.getTalon().setInverted(false);

        // Phoenix6Utility.resetTalonFxFactoryDefaults(m_leftMotor);

        // Phoenix6Utility.applyConfigAndRetry(m_leftMotor, 
        //     () -> m_leftMotor.setControl(new Follower(IO_Right.getTalon().getDeviceID(), true)));
        
    }
    
    /**
    * <h3>setSpeed</h3>
    * @param leftSpeed the speed the left wheel will be set to
    * @param rightSpeed the speed the right wheel will be set to
    */
    public void setSpeed(double leftSpeed, double rightSpeed, Double leftAccel, Double rightAccel) {
        if(leftAccel != null) {
            IO_Left.setSpeed(leftSpeed, leftAccel);
        } else {
            IO_Left.setSpeed(leftSpeed);
        }
        if(rightAccel != null) {
            IO_Right.setSpeed(rightSpeed, rightAccel);    
        } else {
            IO_Right.setSpeed(rightSpeed);
        }
        m_reachedSetPoint = false;
        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/ReachedSetPoint" ,m_reachedSetPoint);
    }

    /**
    * <h3>setSpeed</h3>
    * @param leftSpeed the speed the left wheel will be set to in rot/s
    * @param rightSpeed the speed the right wheel will be set to in rot/s 
    */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        this.setSpeed(leftSpeed, rightSpeed, null, null);
    }

    /**
    * <h3>getLeftMotorSpeed</h3>
    * @return The current motor speed of the left wheel in rps
    */
    public double getLeftMotorSpeed() {
        return IO_Left.getSpeed();
    }

    /**
    * <h3>getRightMotorSpeed</h3>
    * @return The current motor speed of the right wheel in rps
    */
    public double getRightMotorSpeed() {
        return IO_Right.getSpeed();
    }

    /**
    * <h3>getLeftVoltage</h3>
    * @return The current voltage of the left motor
    */
    public double getLeftVoltage() {
        return IO_Left.getVoltage();
    }

    /**
    * <h3>getRightVoltage</h3>
    * @return The current voltage of the right motor
    */
    public double getRightVoltage() {
        return IO_Right.getVoltage();
    }

    /**
    * <h3>getLeftTargetVelocity</h3>
    * @return The current voltage of the right motor
    */
    public double getLeftTargetVelocity() {
        return IO_Left.getTargetVelocity();
    }

    /**
    * <h3>getRightTargetVelocity</h3>
    * @return The current voltage of the right motor
    */
    public double getRightTargetVelocity() {
        return IO_Right.getTargetVelocity();
    }

    /**
    * <h3>stop</h3>
    * This sets the shooter's speed to 0
    */
    public void stop() {
        setSpeed(0.0,0.0,0.0,0.0);
    }

    @Override
    public void periodic() {
        IO_Left.runSim();
        IO_Right.runSim();
        
        Logger.recordOutput(this.getClass().getSimpleName() + "/LeftWheel/Velocity" ,getLeftMotorSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LeftWheel/Voltage" ,getLeftVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LeftWheel/SetPoint" ,getLeftTargetVelocity());

        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/Velocity" ,getRightMotorSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/Voltage" ,getRightVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/SetPoint" ,getRightTargetVelocity());
    }

    public Command newSetSpeedsCommand(double leftSpeed, double rightSpeed) {
        return new InstantCommand(() -> setSpeed(leftSpeed, rightSpeed), this);
    }

    public Command newSetSpeedsCommand(SpeakerScoreUtility speakerUtil) {
        return new InstantCommand(() ->  setSpeed(speakerUtil.getLeftShooterSpeed(), speakerUtil.getRightShooterSpeed()), this);
    }

    public Command newCalcAndSetSpeedsCommand() {
        double speed = SpeakerScoreUtility.computeShooterSpeed(SpeakerScoreUtility.inchesToSpeaker());
        return new InstantCommand(() -> setSpeed(speed, speed));
    }

    public Command shootTo(ShooterAction target) {
        return newSetSpeedsCommand(target.leftWheelSpeed, target.rightWheelSpeed);
    }

    enum ShooterAction {
        SPEAKER(0.7,0.8),
        AMP(-0.3,-0.3),
        EJECT(0.2,0.2);
        
        public final double leftWheelSpeed;
        public final double rightWheelSpeed;

        private ShooterAction(double leftSpeed, double rightSpeed) {
            leftWheelSpeed = leftSpeed;
            rightWheelSpeed = rightSpeed;
        }
    }

    public boolean atSetpoint() {
        m_reachedSetPoint = MathUtil.applyDeadband(getRightTargetVelocity() - getRightMotorSpeed(), VELOCITY_DEADBAND) == 0.0
            && MathUtil.applyDeadband(getLeftTargetVelocity() - getLeftMotorSpeed(),VELOCITY_DEADBAND) == 0.0; 
        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/ReachedSetPoint" ,m_reachedSetPoint);
        return m_reachedSetPoint;
    }

    public Command newWaitUntilSetpointCommand(double timeout) {
        return new WaitCommand(timeout).until(() -> atSetpoint());
    }
}

