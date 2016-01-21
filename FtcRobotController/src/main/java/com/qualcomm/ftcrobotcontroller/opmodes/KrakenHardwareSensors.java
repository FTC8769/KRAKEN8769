package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.TouchSensor;

//------------------------------------------------------------------------------
//
// KrakenHardwareSensors copied from PushBotHardwareSensors (with IR and optical sensors deleted)
//
/**
 * Provides a single sensor access point between custom op-modes and the
 * OpMode class for the Push Bot.  It does this by extending the original Push
 * Bot's hardware and telemetry classes.
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 * @author SSI Robotics
 * @version 2015-08-13-20-04
 */
public class KrakenHardwareSensors  extends Kraken_8769_Autonomous_Base

{
    //--------------------------------------------------------------------------
    //
    // PushBotHardwareSensors
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public KrakenHardwareSensors()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotHardwareSensors


    //--------------------------------------------------------------------------
    //
    // dump the climbers in the bin
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    public void runOpMode() throws InterruptedException {
        super.TurnRight(90);//Turn right 90 degrees
        super.DriveForward(65);//drive forward 65 inches < 3 tiles
        driveForwardUntilTouching();
        super.Sweeper(0);
        // super.releaseClimbers() !!!;
    }

    //--------------------------------------------------------------------------
    //
    // init
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void init ()

    {
        //
        // Use a base class method to associate class members to non-sensor
        // hardware ports (i.e. left/right drive wheels, left arm, etc.).
        //
        super.init ();

        //
        // Connect the sensors.
        //
        try
        {
            v_sensor_touch = hardwareMap.touchSensor.get ("sensor_touch");
        }
        catch (Exception p_exeception)
        {
            v_sensor_touch = null;
        }
    } // init

    //--------------------------------------------------------------------------
    //
    // is_touch_sensor_pressed
    //
    /**
     * Indicate whether the touch sensor has been pressed.
     */
    boolean is_touch_sensor_pressed ()

    {
        boolean l_return = false;

        if (v_sensor_touch != null)
        {
            l_return = v_sensor_touch.isPressed ();
        }

        return l_return;

    } // is_touch_sensor_pressed

    //--------------------------------------------------------------------------
    //
    // move_arm_upward_until_touch
    //
    /**
     * Apply upward power to the arm motor until the touch sensor is pressed.
     */
    boolean driveForwardUntilTouching ()

    {
        //
        // If the touch sensor is pressed, halt the motors.
        //
        if (is_touch_sensor_pressed ())
        {
            super.stopMotors();
        }
        //
        // Move the arm upward at full power.
        //
        else
        {
            super.DriveForward(1);//drive forward 48 inches 2 tiles
        }

        //
        // Return whether the sensor has been pressed.
        //
        return is_touch_sensor_pressed ();

    } // move_arm_upward_until_touch

    private void m_left_arm_power(float v) {

        //spin motor
    }


    //--------------------------------------------------------------------------
    //
    // v_sensor_touch
    //
    /**
     * Manage the touch sensor.
     */
    private TouchSensor v_sensor_touch;



} // PushBotHardwareSensors



