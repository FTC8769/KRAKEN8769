/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class Kraken_8769_Autonomous_Base extends LinearOpMode {

    static String MOTORLF = "mlf"; //Motor left front
    static String MOTORLB = "mlr"; //Motor left rear
    static String MOTORRF = "mrf"; //Motor right front
    static String MOTORRB = "mrr"; //Motor right rear
    static String SERVOBUCKET = "bkt";
    static String MOTORARM = "swx";  //Motor arm

    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorLB;

    DcMotor motorArm;

    Servo servoBucket;

    static int msPerInch = 28;
    static float msPerDegree = (float)8.25;

    float sweeper = 0;
    float liftUD = 0;

    public Kraken_8769_Autonomous_Base() {
    }

    public void runOpMode() throws InterruptedException
    {
        //        telemetry.clearData();
        mapMotors();
        waitForStart();
        try
        {
            DriveForward(10);
            Thread.sleep(500);
            DriveBackwards(9);
            Thread.sleep(10000);
        }
        catch (InterruptedException e) {
            DbgLog.msg( "autonomous code failed!");
            DbgLog.msg( e.getLocalizedMessage() );
            e.printStackTrace();
        }
    }

    public void mapMotors()
    {
        try
        {
            motorLF = hardwareMap.dcMotor.get(MOTORLF);
            motorLF.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            DbgLog.msg( "motorLF is not working!");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            motorLF = null;
        }

        try
        {
            motorLB = hardwareMap.dcMotor.get(MOTORARM);
            motorLB.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            DbgLog.msg( "motorLB is not working!");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            motorLB = null;
        }

        try
        {
            motorRF = hardwareMap.dcMotor.get(MOTORRF);
        }
        catch (Exception p_exeception)
        {
            DbgLog.msg( "motorRF is not working!");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            motorRF = null;
        }

        try
        {
            motorRB = hardwareMap.dcMotor.get(MOTORRB);
        }
        catch (Exception p_exeception)
        {
            DbgLog.msg( "motorRB is not working!");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            motorRB = null;
        }

        try
        {
            motorArm = hardwareMap.dcMotor.get(MOTORARM);
        }
        catch (Exception p_exeception)
        {
            DbgLog.msg( "motorArm is not working!");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            motorArm = null;
        }

        try
        {
            v_sensor_touch = hardwareMap.touchSensor.get ("sensor_touch");
        }

        catch (Exception p_exeception)
        {
            DbgLog.msg( "sensor_touch is not working!");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            v_sensor_touch = null;
        }

        servoBucket = hardwareMap.servo.get(SERVOBUCKET);

    }




    public void ArmSet()
    {
        motorArm.setPower(1);
        try {
            Thread.sleep(500);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void DriveForward(int inches)
    {
        long time = inches * msPerInch;
        motorLF.setPower(.25);
        motorLB.setPower(.25);
        motorRB.setPower(.25);
        motorRF.setPower(.25);
        try {
            Thread.sleep(time);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        stopMotors();
    }

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
    boolean move_arm_upward_until_touch ()

    {
        //
        // If the touch sensor is pressed, halt the motors.
        //
        if (is_touch_sensor_pressed ())
        {
            //m_left_arm_power (0.0f);
        }
        //
        // Move the arm upward at full power.
        //
        else
        {
            //m_left_arm_power (1.0f);
        }

        //
        // Return whether the sensor has been pressed.
        //
        return is_touch_sensor_pressed ();

    } // move_arm_upward_until_touch


    public void DriveBackwards(int inches)
    {
        long time = inches * msPerInch;
        motorLF.setPower(-.25);
        motorLB.setPower(-.25);
        motorRB.setPower(-.25);
        motorRF.setPower(-.25);
        try {
            Thread.sleep(time);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        stopMotors();
    }


    public void TurnRight(float degrees)
    {
        long time = (long) (degrees * msPerDegree);
        motorRB.setPower(-.25);
        motorRF.setPower(-.25);
        motorLF.setPower(.25);
        motorLB.setPower(.25);
        try {
            Thread.sleep(time);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }

        stopMotors();
    }
    public void TurnLeft(float degrees)
    {
        long time = (long) (degrees * msPerDegree);
        motorRB.setPower(.25);
        motorRF.setPower(.25);
        motorLF.setPower(-.25);
        motorLB.setPower(-.25);
        try {
            Thread.sleep(time);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }

        stopMotors();
    }

    public void stopMotors()
    {
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRB.setPower(0);
        motorRF.setPower(0);
        motorArm.setPower(0);
    }

    private TouchSensor v_sensor_touch;

}
