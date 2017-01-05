package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotManual
//
/**
 * Provide a basic manual operational mode that uses the left and right
 * drive motors, left arm motor, servo motors and gamepad input from two
 * gamepads for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class APsManual3 extends PushBotTelemetry

{
    //--------------------------------------------------------------------------
    //
    // PushBotManual
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public APsManual3()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until this method ends.
        //

        //
        // Manage the drive wheel motors.
        //
        float l_left_drive_power = scale_motor_power (-gamepad1.left_stick_y);
        float l_right_drive_power = scale_motor_power (-gamepad1.right_stick_y);

        set_drive_power (l_left_drive_power, l_right_drive_power);

        //
        // Manage the arm motor.
        //
       // float l_left_arm_power = scale_motor_power (-gamepad2.left_stick_y);
       // m_left_arm_power (l_left_arm_power);

        //----------------------------------------------------------------------
        //
        // Servo Motors
        //
        // Obtain the current values of the gamepad 'x' and 'b' buttons.
        //
        // Note that x and b buttons have boolean values of true and false.
        //
        // The clip method guarantees the value never exceeds the allowable range of
        // [0,1].
        //
        // The setPosition methods write the motor power values to the Servo
        // class, but the positions aren't applied until this method ends.
        //
    //    if (gamepad2.x)
    //    {
    //        m_hand_position (a_hand_position () + 0.05);
    //    }
    //    else if (gamepad2.b)
    //    {
    //        m_hand_position (a_hand_position () - 0.05);
    //    }

        if(gamepad1.left_bumper)
        {
            m_hand_position(a_hand_position() + 0.02);
        }
        else if(gamepad1.left_trigger>0)
        {
            m_hand_position(a_hand_position() - 0.02);
        }
        else
        {
                m3_hand_position(.5);
                m2_hand_position(.5);
        }
        if(gamepad2.left_bumper)
    {
        m_left_side_position(a_left_side_position() + .02);
    }
    else if(gamepad2.left_trigger>0)
    {
        m_left_side_position(a_left_side_position() - 0.02);
    }
        if(gamepad2.right_bumper)
        {
            m_right_side_position(a_right_side_position() - .02);
        }
        else if(gamepad2.right_trigger>0)
        {
            m_right_side_position(a_right_side_position() + .02);
        }

        if(gamepad1.right_bumper)
        {
            m_right_arm_power(.35);
            m_left_arm_power(-.35);
        }
        else if(gamepad1.right_trigger>0)
        {
            m_right_arm_power(-.35);
            m_left_arm_power(.35 );
        }
        else
        {
            m_right_arm_power(0);
            m_left_arm_power(0);
        }

        if(gamepad1.dpad_up)
        {

            if (isEngaged)
            {
                m_left_reel_power(-1);
                m_right_reel_power(1);
            }
            else
            {
                m_left_reel_power(1);
                m_right_reel_power(-1);
            }
        }
        else if(gamepad1.dpad_down)
        {


                m_left_reel_power(-1);
                m_right_reel_power(1);

        }
        else
        {
            m_left_reel_power(0);
            m_right_reel_power(0);
        }

        if (gamepad2.x)
        {
            engage_jammer();

        }
        if (gamepad2.a)
        {
            disengage_jammer();
        }



        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        update_gamepad_telemetry ();

    } // loop

} // PushBotManual
