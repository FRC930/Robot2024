///ThriftyNova API v1.0.2

package frc.robot.utilities;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;

public class ThriftyNova
{
    private final int THRIFTY_ID = 13;
    private final int MOTOR_CONTROLLER_ID = 2;

    private final int STATUS_API_CLASS = 20;
    private final int CONTROL_API_CLASS = 10;

    private final int PERCENT_OUTPUT = 1;
    private final int POSITION_INTERNAL = 2;
    private final int VELOCITY_INTERNAL = 3;
    private final int POSITION_QUAD = 4;
    private final int VELOCITY_QUAD = 5;
    // private final int POSITION_CAN = 6; (WIP)
    // private final int VELOCITY_CAN = 7; (WIP)

    private int device_id = -1;
    private int device_handle = -1;

    private double current_set_point = 0;
    private int conversion_factor = 1;

    private enum controller_states
    {
        off,
        percent_output,
        position_internal,
        position_quadrature,
        velocity_internal,
        velocity_quadrature,
        current
    }
    private controller_states controller_state = controller_states.off;

    public enum encoder_types
    {
        internal,
        quadrature,
        can
    }

    public enum current_types
    {
        stator,
        supply
    }

    ///DEVICE SET UP
    public ThriftyNova(int dev_id)
    {
        device_id = dev_id;
        device_handle = CANAPIJNI.initializeCAN          
        (
            THRIFTY_ID, 
            device_id, 
            MOTOR_CONTROLLER_ID
        );
    }
    
    ///SETTERS
    /** 
     * The speed to set the motor. Expected input values are between -1.0 and 1.0.
     */
    public void setPercentOutput(double percent_output)
    {   
        setMotor(0, PERCENT_OUTPUT, translateOutput(limitRange(-1, 1, percent_output)));
        controller_state = controller_states.percent_output;
    }

    /** 
     * Drives the motor towards the given position using the configured P, I, D and F values.
     */
    public void setPosition(double target_position)
    {   
        setPosition(target_position, 0, encoder_types.internal);
    }
    public void setPosition(double target_position, int pid_select, encoder_types encoder_type)
    {   
        current_set_point = target_position;

        int output_type = 0;
        if(encoder_type == encoder_types.internal)
        {
            output_type = POSITION_INTERNAL;
            controller_state = controller_states.position_internal;
        }
        else if(encoder_type == encoder_types.quadrature) 
        {
            output_type = POSITION_QUAD;
            controller_state = controller_states.position_quadrature;
        }

        target_position *= conversion_factor;
        setMotor(pid_select, output_type, (int)target_position);
    }

    /** 
     * Drives the motor at the given velocity using the configured P, I, D and F values.
     */
    public void setVelocity(double target_velocity)
    {   
        setVelocity(target_velocity, 0, encoder_types.internal);
    }
    public void setVelocity(double target_velocity, int pid_select, encoder_types encoder_type)
    {   
        current_set_point = target_velocity;

        int output_type = 0;
        if(encoder_type == encoder_types.internal)
        {
            output_type = VELOCITY_INTERNAL;
            controller_state = controller_states.velocity_internal;
        }
        else if(encoder_type == encoder_types.quadrature) 
        {
            output_type = VELOCITY_QUAD;
            controller_state = controller_states.velocity_quadrature;
        }

        target_velocity *= conversion_factor;
        setMotor(pid_select, output_type, (int)target_velocity);
    }

    ///CONFIGS
    /**
     * If enabled, will drive the motor in reverse when commanded to go forward.
     * Will return true on successful upload.
     */
    public boolean setInverted(boolean inverted) {return setConfig(0, inverted ? 1 : 0);}

    /**
     * If enabled, will turn brake mode on. Disabling will set to coast mode.
     * Will return true on successful upload.
     */
    public boolean setBrakeMode(boolean brake_mode) {return setConfig(1, brake_mode ? 1 : 0);}
    
    /**
     * Sets the max forward speed which the motor will not exceed, even if instructed to.
     * Input ranges from 0 to 1. Will return true on successful upload.
     */
    public boolean setMaxForward(double max_forward) 
    {
        return setConfig(3, translateOutput(limitRange(0, 1, max_forward)));
    }

    /**
     * Sets the max reverse speed which the motor will not exceed, even if instructed to.
     * Input ranges from 0 to -1. Will return true on successful upload.
     */
    public boolean setMaxReverse(double max_reverse) 
    {
        return setConfig(4, translateOutput(-limitRange(-1, 0, max_reverse)));
    }

    /**
     * Sets the ramp rate in seconds. For example, an input of .5 will ramp the motor from idle to 100%
     * over the course of .5 seconds. Input ranges from 0 to 10. Will return true on successful upload.
     */
    public boolean setRampUp(double ramp_up) 
    {
        ramp_up = limitRange(0, 10, ramp_up);

        double translated_ramp = 1;

        if(ramp_up != 0) translated_ramp = 1 / (ramp_up * 1000);    

        return setConfig(5, translateOutput(translated_ramp));
    }

    /**
     * Sets the ramp rate in seconds. For example, an input of .5 will ramp the motor from 100% to idle
     * over the course of .5 seconds. Input ranges from 0 to 10. Will return true on successful upload.
     */
    public boolean setRampDown(double ramp_down) 
    {
        ramp_down = limitRange(0, 10, ramp_down);

        double translated_ramp = 1;

        if(ramp_down != 0) translated_ramp = 1 / (ramp_down * 1000);   

        return setConfig(6, translateOutput(translated_ramp));
    }

    /**
     * Will set the max current the motor can draw in amps. Motor speeds will be capped to satisfy the max current. 
     * Will return true on successful upload.
     */
    public boolean setMaxCurrent(double max_current) {return setConfig(7, translateSFOutput(max_current));}

    /**
     * Will change what max current uses for current limiting calculations.
     * 
     * Stator: Uses the total draw of phase a, b and c.
     * Supply: Uses stator plus the draw of the controller itself.
     * 
     * Will return true on successful upload.
     */
    public boolean setMaxCurrentType(current_types current_type)
    {
        return setConfig(8, current_type == current_types.stator ? 0 : 1);
    }

    /*
     * Sets the ID to listen to for an external encoder. (WIP)
     */
    // public boolean setExtSensorId(int extern_id) {return setConfig(9, extern_id);}

    public boolean setkP(double p) {return setkP(p, 0);}
    public boolean setkP(double p, int slot) 
    {
        return setConfig(slot == 1 ? 14 : 10, translatePIDOutput(p));
    }
    public boolean setkI(double i) {return setkI(i, 0);}
    public boolean setkI(double i, int slot) 
    {
        return setConfig(slot == 1 ? 15 : 11, translatePIDOutput(i));
    }
    public boolean setkD(double d) {return setkD(d, 0);}
    public boolean setkD(double d, int slot) 
    {
        return setConfig(slot == 1 ? 16 : 12, translatePIDOutput(d));
    }
    public boolean setkF(double f) {return setkF(f, 0);}
    public boolean setkF(double f, int slot) 
    {
        return setConfig(slot == 1 ? 17 : 13, translatePIDOutput(f));
    }


    ///Status Frame control
    /**
     * Sets the frequency of sent CAN frames for faults in seconds.
     */
    public boolean setSFFaults(double period) {return setConfig(18, translateSFOutput(period));}

    /**
     * Sets the frequency of sent CAN frames for the internal encoder in seconds.
     */
    public boolean setSFSensor(double period) {return setConfig(19, translateSFOutput(period));}

    /**
     * Sets the frequency of sent CAN frames for the quadrature encoder in seconds.
     */
    public boolean setSFQuadSensor(double period) {return setConfig(20, translateSFOutput(period));}

    /**
     * Sets the frequency of sent CAN frames for motor control in seconds.
     */
    public boolean setSFControl(double period) {return setConfig(21, translateSFOutput(period));}

    /**
     * Sets the frequency of sent CAN frames for current readings in seconds.
     */
    public boolean setSFCurrent(double period) {return setConfig(22, translateSFOutput(period));}

    /**
     * Sets the soft limits that the motor will not go past if enabled. 
     */
    public boolean setSoftLimits(double rev_limit, double fwd_limit)
    {
        rev_limit *= conversion_factor;
        fwd_limit *= conversion_factor;

        return (setConfig(26, (int)rev_limit) & setConfig(25, (int)fwd_limit));
    }
    /**
     * Will enable / disable soft limits.
     */
    public boolean enableSoftLimits(boolean enable) {return setConfig(24, enable ? 1 : 0);}
    /**
     * Will enable / disable hard limits.
     */
    public boolean enableHardLimits(boolean enable) {return setConfig(23, enable ? 1 : 0);}

    /**
     * Sets the conversion factor for reading and writing anything to do with any encoder position.
     * This includes setting the encoder position, setting target position, setting target velocity, 
     * getting position, getting velocity, getting the closed loop error and setting the soft limits. 
     * 
     * For example, if you want to set a Neo550 to fully rotate 5 times when you set the position to 5,
     * you would set the conversion factor to 42 (the amount of encoder tics it takes a Neo550 to rotate once). 
     * 
     * Can not set lower that 1. Math is (encoder_tics / conversion_factor) == output
     */
    public void setConversionFactor(int factor) {conversion_factor = (int)limitRange(1, 2147454607, factor);}


    ///GETTERS   
    /**
     * Returns the position of the motor. Can also specify the encoder type.
     */ 
    public double getPosition() {return getPosition(encoder_types.internal);}
    public double getPosition(encoder_types encoder_type) 
    {
        return getMotorStatus(encoder_type == encoder_types.internal ? 1 : 2, 4, 7) / (double)conversion_factor;
    }
    /**
     * Returns the velocity of the motor. Can also specify the encoder type.
     */ 
    public double getVelocity() {return getVelocity(encoder_types.internal);}
    public double getVelocity(encoder_types encoder_type) 
    {
        return getMotorStatus(encoder_type == encoder_types.internal ? 1 : 2, 0, 3) / (double)conversion_factor;
    }
    /**
     * Returns the current draw of motor in milliamps.
     */ 
    public int getStatorCurrent() {return getMotorStatus(4, 0, 1);}

    /**
     * Returns the current draw of motor plus the controller itself in milliamps.
     */
    public int getSupplyCurrent() {return getMotorStatus(4, 6, 7);}

    /**
     * Returns the last set point that was input. 
     * (Position or velocity)
     */ 
    public double getSetPoint() {return current_set_point;}

    /**
     * Returns the error to the last set point that was input. 
     * (Position or velocity, internal or external)
     */ 
    public double getClosedLoopErr()
    {
        switch(controller_state) 
        {
            case position_internal:
                return current_set_point - getPosition();
            case position_quadrature:
                return current_set_point - getPosition(encoder_types.quadrature);
            case velocity_internal:
                return current_set_point - getVelocity();
            case velocity_quadrature:
                return current_set_point - getVelocity(encoder_types.quadrature);
            case off:
            case percent_output:
            case current:
            default:
                return 0;
        }
    }

    /**
     * Returns the CAN ID of the motor controller.
     */ 
    public int getID() {return device_id;}


    ///UTIL
    private void setMotor(int pid_select, int control_type, int target)
    {
        CANAPIJNI.writeCANPacketNoThrow(
            device_handle, 
            new byte[] {
                0, 
                0, 
                (byte)pid_select, 
                (byte)control_type, 
                (byte)((target >> 24) & 0xFF), 
                (byte)((target >> 16) & 0xFF), 
                (byte)((target >> 8) & 0xFF), 
                (byte)(target & 0xFF)
            },
            getApiId(CONTROL_API_CLASS, 1)
        );
    }

    private boolean setConfig(int index, int value)
    {     
        if(CANAPIJNI.writeCANPacketNoThrow(
            device_handle, 
            new byte[] {
                (byte)((value >> 24) & 0xFF), 
                (byte)((value >> 16) & 0xFF), 
                (byte)((value >> 8) & 0xFF), 
                (byte)(value & 0xFF),
                (byte)((index >> 24) & 0xFF), 
                (byte)((index >> 16) & 0xFF), 
                (byte)((index >> 8) & 0xFF), 
                (byte)(index & 0xFF)
            },
            getApiId(CONTROL_API_CLASS, 0)
        ) != 0) return false;

        CANData data_in = new CANData();
        long rio_time = RobotController.getFPGATime();

        ///Checks if robot disabled because 1. blocking isn't as much of an issue and 2. rio start up times
        while((RobotController.getFPGATime() - rio_time) < (RobotState.isDisabled() ? 50000 : 5000))
        {
            CANAPIJNI.readCANPacketNew(
                device_handle, 
                getApiId(STATUS_API_CLASS, 5), 
                data_in
            );

            if(data_in.data[0] == 0x69) return true;
            //6E696365
        }

        System.out.println("\nThriftyWarning: Failed to set configuration " + index + ".\n");

        return false;
    }

    /**
     * Sets the position of the encoder. Can also pass in a second argument to specify the encoder type.
     */
    public void setEncoderPosition(double value) {setEncoderPosition(value, encoder_types.internal);}
    public void setEncoderPosition(double value, encoder_types encoder_type)
    {
        value *= conversion_factor;

        CANAPIJNI.writeCANPacketNoThrow(
            device_handle, 
            new byte[] {
                (byte)(((int)value >> 24) & 0xFF), 
                (byte)(((int)value >> 16) & 0xFF), 
                (byte)(((int)value >> 8) & 0xFF), 
                (byte)((int)value & 0xFF),
                (byte)(encoder_type == encoder_types.internal ? 0 : 1), 
                (byte)(0), 
                (byte)(0), 
                (byte)(0)
            },
            getApiId(CONTROL_API_CLASS, 2)
        );
    }

    private int getMotorStatus(int index, int start_byte, int end_byte)
    {
        CANData data_in = new CANData();
        CANAPIJNI.readCANPacketLatest(
            device_handle, 
            getApiId(STATUS_API_CLASS, index), 
            data_in
        );

        int motor_data = 0;

        for(int i = start_byte; i <= end_byte; i++)
        {
            motor_data += (data_in.data[i] & 0xFF) << ((end_byte - i) * 8);
        }

        return motor_data;
    }

    private int getApiId(int api_class, int api_index)
    {
        return ((api_class << 4) + api_index);
    }

    private int translateSFOutput(double input)
    {
        return (int)(1000 * input);
    }

    private int translateOutput(double input)
    {
        return (int)(10000 * input);
    }

    private int translatePIDOutput(double input)
    {
        return (int)(1000000 * input);
    }

    private double limitRange(double min, double max, double input)
    {
        if(input < min) return min;
        if(input > max) return max;

        return input;
    }
}
