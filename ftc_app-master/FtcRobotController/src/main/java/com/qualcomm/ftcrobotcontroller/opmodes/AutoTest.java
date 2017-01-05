package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotAuto
//

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders implemented using a state machine for
 * the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class AutoTest extends PushBotHardwareSensors

{
    //--------------------------------------------------------------------------
    //
    // PushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
  //  DcMotor lego1;
  //  DcMotor lego2;
    DcMotor rightMotor;
    DcMotor leftMotor;

    final static int ENCODER_CPR = 1440;
    final static double GEAR_RATIO = 1;
    final static int WHEEL_DIAMETER = 4;
    final static int DISTANCE = 72;

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
    final static double COUNTS = ENCODER_CPR*ROTATIONS*GEAR_RATIO;

    public AutoTest()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotAuto

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */

    public void init()
    {
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftMotor = hardwareMap.dcMotor.get("left_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    @Override public void start ()

    {

     //   lego1= hardwareMap.dcMotor.get("lego_1");
     //   lego2= hardwareMap.dcMotor.get("lego_2");

        leftMotor.setTargetPosition((int) COUNTS);
        rightMotor.setTargetPosition((int)COUNTS);

        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(.5);
        rightMotor.setPower(.5);

        if(has_left_drive_encoder_reached(COUNTS))
        {
            leftMotor.setPower(.5);
            rightMotor.setPower(-.5);
        }

        //
        // Call the PushBotHardware (super/base class) start method.
        //
        super.start ();

        //
        // Reset the motor encoders on the drive wheels.
        //
   //     reset_drive_encoders ();

    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and encoder input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {

        if(is_touch_sensor_pressed()){
            float l_left_drive_power = scale_motor_power (1f);
            float l_right_drive_power = scale_motor_power (1f);

            set_drive_power (l_left_drive_power, l_right_drive_power);
            telemetry.addData("isPressed", String.valueOf(is_touch_sensor_pressed()));
       //     lego1.setPower(l_left_drive_power);
       //     lego2.setPower(l_right_drive_power);
        }
        else{
            set_drive_power(0,0);
        }

   //     if (a_ods_light_detected()>.1){
   //         set_drive_power(1f,1f);
  //          telemetry.addData("Light Detected: ", String.valueOf(a_ods_light_detected()));
   //     }
    //    else {
   //         set_drive_power(0, 0);
   //         telemetry.addData("Not enough light detected: ", String.valueOf(a_ods_light_detected()));
   //     }
        update_telemetry();


    }

} // PushBotAuto
