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
public class APSAuto3 extends PushBotHardwareSensors

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

    public APSAuto3()

    {


    } // PushBotAuto


    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */



    @Override public void start ()
    {

      //  leftFrontMotor.setPower(.5);
      //  rightFrontMotor.setPower(.5);



        //
        // Call the PushBotHardware (super/base class) start method.
        //
       // super.start ();



    } // start


    @Override public void loop ()

    {



        set_drive_power(.5, .5);
        try{
            Thread.sleep(4000);
        }catch(InterruptedException e){}
        set_drive_power(.5, -.5);
        try{
            Thread.sleep(500);
        }catch(InterruptedException e){}
        set_drive_power(.5, .5);
        try{
            Thread.sleep(500);
        }catch(InterruptedException e){}
        set_drive_power(0,0);




        //   if (a_left_encoder_count() > 8000) {
          //      set_drive_power(0, 0);
         //   }


            telemetry.addData("isPressed", String.valueOf(is_touch_sensor_pressed()));
            telemetry.addData("Left Position: ", leftMotor.getCurrentPosition());
            telemetry.addData("Right Position: ", rightMotor.getCurrentPosition());




        update_telemetry();


    }

} // PushBotAuto
