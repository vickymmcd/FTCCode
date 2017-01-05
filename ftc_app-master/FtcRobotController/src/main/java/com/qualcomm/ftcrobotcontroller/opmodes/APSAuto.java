package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotAuto
//

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders implemented using a state machine for
 * the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class APSAuto extends PushBotHardwareSensors

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
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;

    final static int ENCODER_CPR = 1440;
    final static double GEAR_RATIO = 1;
    final static int WHEEL_DIAMETER = 4;
    final static int DISTANCE = 72;
    ElapsedTime time;

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
    final static double COUNTS = ENCODER_CPR*ROTATIONS*GEAR_RATIO;

    public APSAuto()

    {


    } // PushBotAuto


    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */

    public void init()
    {

    }

    @Override public void start ()
    {

        leftMotor.setTargetPosition((int) COUNTS);
        rightMotor.setTargetPosition((int) COUNTS);

        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
      //  leftFrontMotor.setPower(.5);
      //  rightFrontMotor.setPower(.5);


        if (rightMotor.getCurrentPosition()>8000) {
            leftMotor.setPower(-1);
            rightMotor.setPower(1);
            double currentTime = System.nanoTime();
            if (System.nanoTime()-currentTime > 1) {
                leftMotor.setPower(1);
                rightMotor.setPower(1);
            }
        }


        if(is_touch_sensor_pressed()) {
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            double currentHandPos = a_hand_position();
            open_hand();
            m_hand_position(currentHandPos);

            double currentTime2 = System.nanoTime();
            while (System.nanoTime() - currentTime2 < 1) {
                rightMotor.setPower(-.5);
                leftMotor.setPower(.5);
            }

            leftMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

            leftMotor.setTargetPosition(4320);
            rightMotor.setTargetPosition(4320);

            leftMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
            rightMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

            rightMotor.setPower(.5);
            leftMotor.setPower(.5);
        }


        //
        // Call the PushBotHardware (super/base class) start method.
        //
       // super.start ();



    } // start


    @Override public void loop ()

    {



            telemetry.addData("isPressed", String.valueOf(is_touch_sensor_pressed()));
            telemetry.addData("Left Position: ", leftMotor.getCurrentPosition());
            telemetry.addData("Right Position: ", rightMotor.getCurrentPosition());




        update_telemetry();


    }

} // PushBotAuto
