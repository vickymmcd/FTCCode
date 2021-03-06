package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
//
// PushBotHardware
//
/**
 * Provides a single hardware access point between custom op-modes and the
 * OpMode class for the Push Bot.
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 * @author SSI Robotics
 * @version 2015-08-13-20-04
 */
public class PushBotHardware extends OpMode

{
    //--------------------------------------------------------------------------
    //
    // PushBotHardware
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotHardware ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotHardware

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
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).
        //
        // The variable below is used to provide telemetry data to a class user.
        //
        v_warning_generated = false;
        v_warning_message = "Can't map; ";

        //
        // Connect the drive wheel motors.
        //
        // The direction of the right motor is reversed, so joystick inputs can
        // be more generically applied.
        //
        try
        {
            v_motor_left_drive = hardwareMap.dcMotor.get ("left_drive");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("no left_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_left_drive = null;
        }
        try
        {
            v_motor_left_front = hardwareMap.dcMotor.get ("left_front");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("no left_front");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_left_front = null;
        }

        try
        {
            v_motor_right_drive = hardwareMap.dcMotor.get ("right_drive");
            v_motor_right_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("no right_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_right_drive = null;
        }
        try
        {
            v_motor_right_front = hardwareMap.dcMotor.get ("right_front");
            v_motor_right_front.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("no right_front");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_right_front = null;
        }

        //
        // Connect the arm motor.
        //
        //try
        //{
        //    v_motor_left_arm = hardwareMap.dcMotor.get ("left_arm");
        //}
        //catch (Exception p_exeception)
        //{
        //    m_warning_message ("left_arm");
        //    DbgLog.msg (p_exeception.getLocalizedMessage ());

        //    v_motor_left_arm = null;
        //}
        //try
        //{
        //    v_motor_right_arm = hardwareMap.dcMotor.get ("right_arm");
        //}
        //catch (Exception p_exeception)


        try
        {
            v_left_reel = hardwareMap.dcMotor.get ("left_reel");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_reel");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_left_reel = null;
        }
        try
        {
            v_right_reel = hardwareMap.dcMotor.get ("right_reel");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_reel");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_right_reel = null;
        }

        //
        // Connect the servo motors.
        //

        try
        {
            v_servo_sensor = hardwareMap.servo.get ("sensor_servo");
            v_servo_sensor.setPosition (.86);
        }
        catch (Exception p_exeception) {
            m_warning_message("servo sensor");
            DbgLog.msg(p_exeception.getLocalizedMessage());

            v_servo_sensor = null;
        }
        try
        {
            v_servo_left_hand = hardwareMap.servo.get ("left_hand"); //winch elevation servo
            v_servo_left_hand.setPosition (Servo.MIN_POSITION);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_left_hand = null;
        }
        // Indicate the initial position of both the left and right servos.  The
        // hand should be halfway opened/closed.
        //
        double l_hand_position = 0;

        try
        {
            v_servo_left_side = hardwareMap.servo.get ("left_side");  //
            //      v_servo_left_hand.setPosition (l_hand_position);
            v_servo_left_side.setPosition(.67);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_left_side = null;
        }
        try
        {
            v_servo_right_side = hardwareMap.servo.get ("right_side");
            //      v_servo_left_hand.setPosition (l_hand_position);
            v_servo_right_side.setPosition(.25);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_side");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_right_side = null;
        }
        try
        {
            v_servo_right_jammer = hardwareMap.servo.get ("right_jammer");
            v_servo_right_jammer.setPosition(.32);
            //      v_servo_left_hand.setPosition (l_hand_position);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right jammer");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_right_jammer = null;
        }
        try
        {
            v_servo_left_jammer = hardwareMap.servo.get ("left_jammer");
            v_servo_left_jammer.setPosition(.47);
            //      v_servo_left_hand.setPosition (l_hand_position);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left jammer");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_left_jammer = null;
        }
        try
        {
            v_servo_flagtop = hardwareMap.servo.get ("tflag_servo");
            v_servo_flagtop.setPosition (Servo.MAX_POSITION);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("top_flag_servo");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_flagtop = null;
        }
        try
        {
            v_servo_flagbottom = hardwareMap.servo.get ("bflag_servo");
            v_servo_flagbottom.setPosition (Servo.MAX_POSITION);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("bottom_flag_servo");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_flagbottom = null;
        }

        try
        {
            v_servo_climberbottom = hardwareMap.servo.get ("bclimber_servo");
            v_servo_climberbottom.setPosition (.98);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("bottom climber_servo");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_climberbottom = null;
        }
        try
        {
            v_servo_right_hand = hardwareMap.servo.get ("right_hand");
            v_servo_right_hand.setPosition (Servo.MAX_POSITION);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_right_hand = null;
        }
        winch_sensor = hardwareMap.colorSensor.get("winch_sensorx52");
        winch_sensor.setI2cAddress(0x52);
        winch_sensor.enableLed(true);

    } // init

    //--------------------------------------------------------------------------
    //
    // a_warning_generated
    //
    /**
     * Access whether a warning has been generated.
     */
    boolean a_warning_generated ()

    {
        return v_warning_generated;

    } // a_warning_generated

    //--------------------------------------------------------------------------
    //
    // a_warning_message
    //
    /**
     * Access the warning message.
     */
    String a_warning_message ()

    {
        return v_warning_message;

    } // a_warning_message

    //--------------------------------------------------------------------------
    //
    // m_warning_message
    //
    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     *
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void m_warning_message (String p_exception_message)

    {
        if (v_warning_generated)
        {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void start ()

    {
        //
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Perform any actions that are necessary while the OpMode is running.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        //
        // Only actions that are common to all OpModes (i.e. both auto and\
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // loop

    //--------------------------------------------------------------------------
    //
    // stop
    //
    /**
     * Perform any actions that are necessary when the OpMode is disabled.
     *
     * The system calls this member once when the OpMode is disabled.
     */
    @Override public void stop ()
    {
        //
        // Nothing needs to be done for this method.
        //

    } // stop

    //--------------------------------------------------------------------------
    //
    // scale_motor_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    float scale_motor_power (float p_power)
    {
        //
        // Assume no scaling.
        //
        float l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        float l_power = Range.clip (p_power, -1, 1);

        float[] l_array =
                { 0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int)(l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // scale_motor_power

    //--------------------------------------------------------------------------
    //
    // a_left_drive_power
    //
    /**
     * Access the left drive motor's power level.
     */
    double a_left_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_left_drive != null)
        {
            l_return = v_motor_left_drive.getPower ();
        }

        return l_return;

    } // a_left_drive_power

    //--------------------------------------------------------------------------
    //
    // a_right_drive_power
    //
    /**
     * Access the right drive motor's power level.
     */
    double a_right_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_right_drive != null)
        {
            l_return = v_motor_right_drive.getPower ();
        }

        return l_return;

    } // a_right_drive_power

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    void set_drive_power (double p_left_power, double p_right_power)

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setPower (p_left_power);
        }
        if (v_motor_left_front != null)
        {
            v_motor_left_front.setPower (p_left_power);
        }
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setPower (p_right_power);
        }
        if (v_motor_right_front != null)
        {
            v_motor_right_front.setPower (p_right_power);
        }

    } // set_drive_power

    //--------------------------------------------------------------------------
    //
    // run_using_left_drive_encoder
    //
    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setChannelMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_right_drive_encoder
    //
    /**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setChannelMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_encoders
    //
    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_using_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_using_left_drive_encoder ();
        run_using_right_drive_encoder ();

    } // run_using_encoders

    //--------------------------------------------------------------------------
    //
    // run_without_left_drive_encoder
    //
    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            if (v_motor_left_drive.getChannelMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_left_drive.setChannelMode
                        ( DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_right_drive_encoder
    //
    /**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            if (v_motor_right_drive.getChannelMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_right_drive.setChannelMode
                        ( DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_drive_encoders
    //
    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_without_drive_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_without_left_drive_encoder ();
        run_without_right_drive_encoder ();

    } // run_without_drive_encoders

    //--------------------------------------------------------------------------
    //
    // reset_left_drive_encoder
    //
    /**
     * Reset the left drive wheel encoder.
     */
    public void reset_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setChannelMode
                    ( DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_right_drive_encoder
    //
    /**
     * Reset the right drive wheel encoder.
     */
    public void reset_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setChannelMode
                    ( DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_drive_encoders
    //
    /**
     * Reset both drive wheel encoders.
     */
    public void reset_drive_encoders ()

    {
        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_left_drive_encoder ();
        reset_right_drive_encoder ();

    } // reset_drive_encoders

    //--------------------------------------------------------------------------
    //
    // a_left_encoder_count
    //
    /**
     * Access the left encoder's count.
     */
    int a_left_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_left_drive != null)
        {
            l_return = v_motor_left_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_left_encoder_count

    //--------------------------------------------------------------------------
    //
    // a_right_encoder_count
    //
    /**
     * Access the right encoder's count.
     */
    int a_right_encoder_count ()

    {
        int l_return = 0;

        if (v_motor_right_drive != null)
        {
            l_return = v_motor_right_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_right_encoder_count

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reached
    //
    /**
     * Indicate whether the left drive motor's encoder has reached a value.
     */
    boolean has_left_drive_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_left_drive != null)
        {
            //
            // Has the encoder reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_left_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reached
    //
    /**
     * Indicate whether the right drive motor's encoder has reached a value.
     */
    boolean has_right_drive_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_right_drive != null)
        {
            //
            // Have the encoders reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_right_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean have_drive_encoders_reached
    ( double p_left_count
            , double p_right_count
    )

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (has_left_drive_encoder_reached (p_left_count) &&
                has_right_drive_encoder_reached (p_right_count))
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_encoders_reached

    //--------------------------------------------------------------------------
    //
    // drive_using_encoders
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean drive_using_encoders
    ( double p_left_power
            , double p_right_power
            , double p_left_count
            , double p_right_count
    )

    {
        //
        // Assume the encoders have not reached the limit.
        //
        boolean l_return = false;

        //
        // Tell the system that motor encoders will be used.
        //
        run_using_encoders ();

        //
        // Start the drive wheel motors at full power.
        //
        set_drive_power (p_left_power, p_right_power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached (p_left_count, p_right_count))
        {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders ();

            //
            // Stop the motors.
            //
            set_drive_power (0.0f, 0.0f);

            //
            // Transition to the next state when this method is called
            // again.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // drive_using_encoders

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_left_drive_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the left encoder reached zero?
        //
        if (a_left_encoder_count () == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_right_drive_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the right encoder reached zero?
        //
        if (a_right_encoder_count () == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reset
    //
    /**
     * Indicate whether the encoders have been completely reset.
     */
    boolean have_drive_encoders_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached zero?
        //
        if (has_left_drive_encoder_reset () && has_right_drive_encoder_reset ())
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_drive_encoders_reset

    //--------------------------------------------------------------------------
    //
    // a_left_arm_power
    //
    /**
     * Access the left arm motor's power level.
     */
    double a_left_arm_power ()
    {
        double l_return = 0.0;

        if (v_motor_left_arm != null)
        {
            l_return = v_motor_left_arm.getPower ();
        }

        return l_return;

    } // a_left_arm_power

    double a_right_arm_power ()
    {
        double l_return = 0.0;

        if (v_motor_right_arm != null)
        {
            l_return = v_motor_right_arm.getPower ();
        }

        return l_return;

    }

    //--------------------------------------------------------------------------
    //
    // m_left_arm_power
    //
    /**
     * Access the left arm motor's power level.
     */
    void m_left_arm_power (double p_level)
    {
        if (v_motor_left_arm != null)
        {
            v_motor_left_arm.setPower (p_level);
        }


    } // m_left_arm_power
    void m_right_arm_power (double p_level)
    {
        if (v_motor_right_arm != null)
        {
            v_motor_right_arm.setPower (p_level);
        }


    }

    void m_left_reel_power (double p_level)
    {
        if (v_left_reel != null)
        {
            v_left_reel.setPower(p_level);
        }


    }
    void m_right_reel_power (double p_level)
    {
        if (v_right_reel != null)
        {
            v_right_reel.setPower (p_level);
        }


    }


    //--------------------------------------------------------------------------
    //
    // a_hand_position
    //
    /**
     * Access the hand position.
     */
    double a_hand_position ()
    {
        double l_return = 0.0;

        if (v_servo_left_hand != null)
        {
            l_return = v_servo_left_hand.getPosition ();
        }

        return l_return;

    } // a_hand_position
    double a2_hand_position ()
    {
        double l_return = 0.0;

        if (v_servo_right_hand != null)
        {
            l_return = v_servo_right_hand.getPosition ();
        }

        return l_return;

    }
    double a3_hand_position ()
    {
        double l_return = 0.0;

        if (v_servo_left_hand != null)
        {
            l_return = v_servo_left_hand.getPosition ();
        }

        return l_return;

    }
    double a_sensor_servo_position()
    {
        double l_return = 0.0;

        if (v_servo_sensor != null)
        {
            l_return = v_servo_sensor.getPosition ();
        }

        return l_return;
    }

    double a_left_side_position ()
    {
        double l_return = 0.0;

        if (v_servo_left_side != null)
        {
            l_return = v_servo_left_side.getPosition ();
        }

        return l_return;

    }

    double a_right_side_position ()
    {
        double l_return = 0.0;

        if (v_servo_right_side != null)
        {
            l_return = v_servo_right_side.getPosition ();
        }

        return l_return;

    }
    double a_topflag_position ()
    {
        double l_return = 0.0;

        if (v_servo_flagtop != null)
        {
            l_return = v_servo_flagtop.getPosition ();
        }

        return l_return;

    }
    double a_bottomflag_position ()
    {
        double l_return = 0.0;

        if (v_servo_flagbottom != null)
        {
            l_return = v_servo_flagbottom.getPosition ();
        }

        return l_return;

    }
    double a_topclimber_position ()
    {
        double l_return = 0.0;

        if (v_servo_climbertop != null)
        {
            l_return = v_servo_climbertop.getPosition ();
        }

        return l_return;

    }
    double a_bottomclimber_position ()
    {
        double l_return = 0.0;

        if (v_servo_climberbottom != null)
        {
            l_return = v_servo_climberbottom.getPosition ();
        }

        return l_return;

    }

    //--------------------------------------------------------------------------
    //
    // m_hand_position
    //
    /**
     * Mutate the hand position.
     */
    void m_hand_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );

        //
        // Set the value.  The right hand value must be opposite of the left
        // value.
        //
        if (v_servo_left_hand != null)
        {
            v_servo_left_hand.setPosition (l_position);
        }


    } // m_hand_position

    void m2_hand_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_right_hand != null)
        {
            v_servo_right_hand.setPosition(l_position);
        }
    }
    void m3_hand_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_left_hand != null)
        {
            v_servo_left_hand.setPosition(l_position);
        }
    }
    void m_sensor_servo_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_sensor != null)
        {
            v_servo_sensor.setPosition(l_position);
        }
    }


    void m_right_side_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_right_side != null)
        {
            v_servo_right_side.setPosition (l_position);
        }
    }

    void m_left_side_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_left_side != null)
        {
            v_servo_left_side.setPosition (l_position);
        }
    }
    void m_topflag_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_flagtop != null)
        {
            v_servo_flagtop.setPosition (l_position);
        }
    }
    void m_bottomflag_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_flagbottom != null)
        {
            v_servo_flagbottom.setPosition (l_position);
        }
    }
    void m_topclimber_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_climbertop != null)
        {
            v_servo_climbertop.setPosition (l_position);
        }
    }
    void m_bottomclimber_position(double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        if (v_servo_climberbottom != null)
        {
            v_servo_climberbottom.setPosition (l_position);
        }
    }

    //--------------------------------------------------------------------------
    //
    // open_hand
    //
    /**
     * Open the hand to its fullest.
     */
    void open_hand ()

    {
        //
        // Set the value.  The right hand value must be opposite of the left
        // value.
        //
        if (v_servo_left_hand != null)
        {
            v_servo_left_hand.setPosition (Servo.MAX_POSITION);
        }
        if (v_servo_right_hand != null)
        {
            v_servo_right_hand.setPosition (Servo.MIN_POSITION);
        }


    } // open_hand
    public boolean isEngaged = false;

    void engage_jammer ()
    {
        if (v_servo_right_jammer != null)
        {
            v_servo_right_jammer.setPosition (Servo.MAX_POSITION);
        }
        if (v_servo_left_jammer != null)
        {
            v_servo_left_jammer.setPosition (Servo.MIN_POSITION);
        }
        isEngaged = true;
    }
    void disengage_jammer ()
    {
        if (v_servo_right_jammer != null)
        {
            v_servo_right_jammer.setPosition(.32);
        }
        if (v_servo_left_jammer != null)
        {
            v_servo_left_jammer.setPosition(.47);
        }

        isEngaged =false;

    }

    double winch_red_value()
    {
        return winch_sensor.red();
    }
    double winch_blue_value()
    {
        return winch_sensor.blue();
    }
    double winch_green_value()
    {
        return winch_sensor.green();
    }

    //--------------------------------------------
    // ------------------------------
    //
    // v_warning_generated
    //
    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean v_warning_generated = false;

    //--------------------------------------------------------------------------
    //
    // v_warning_message
    //
    /**
     * Store a message to the user if one has been generated.
     */
    private String v_warning_message;

    //--------------------------------------------------------------------------
    //
    // v_motor_left_drive
    //
    /**
     * Manage the aspects of the left drive motor.
     */
    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_left_front;

    //--------------------------------------------------------------------------
    //
    // v_motor_right_drive
    //
    /**
     * Manage the aspects of the right drive motor.
     */
    private DcMotor v_motor_right_drive;
    private DcMotor v_motor_right_front;

    //--------------------------------------------------------------------------
    //
    // v_motor_left_arm
    //
    /**
     * Manage the aspects of the left arm motor.
     */
    private DcMotor v_motor_left_arm;
    private DcMotor v_motor_right_arm;
    private DcMotor v_left_reel;
    private DcMotor v_right_reel;

    //--------------------------------------------------------------------------
    //
    // v_servo_left_hand
    //
    /**
     * Manage the aspects of the left hand servo.
     */
    private Servo v_servo_left_hand;
    private Servo v_servo_left_side;
    private Servo v_servo_right_side;
    private Servo v_servo_right_jammer;
    private Servo v_servo_left_jammer;
    private Servo v_servo_flagbottom;
    private Servo v_servo_flagtop;
    private Servo v_servo_climberbottom;
    private Servo v_servo_climbertop;
    private Servo v_servo_sensor;

    //--------------------------------------------------------------------------
    //
    // v_servo_right_hand
    //
    /**
     * Manage the aspects of the right hand servo.
     */
    private Servo v_servo_right_hand;
    private ColorSensor winch_sensor;

} // PushBotHardware