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
public class APSAuto2 extends PushBotHardwareSensors


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


    public APSAuto2()


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



        //try {
        //    Thread.sleep(1000);                   //1000 milliseconds is one second.
        //} catch(InterruptedException ex) {
        //    Thread.currentThread().interrupt();
        //}

        //
        // Call the PushBotHardware (super/base class) start method.
        //
        // super.start ();






    } // start
    @Override public void loop(){
        while (leftMotor.getCurrentPosition()<=2880) {
            leftMotor.setPower(1);
            leftFrontMotor.setPower(1) ;
            rightMotor.setPower(-1);
            rightFrontMotor.setPower(-1);
        }
        while (leftMotor.getCurrentPosition()<=8000){
            leftMotor.setPower(1);
            leftFrontMotor.setPower(1);
            rightMotor.setPower(1);
            rightFrontMotor.setPower(1);
        }
        leftMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightMotor.setPower(0);
        rightFrontMotor.setPower(0);

        while (true){}
    }




} // PushBotAuto