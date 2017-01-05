package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
public class APSAutoRealLeft extends PushBotHardwareSensors2 {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;

    @Override
    public void runOpMode() throws InterruptedException {


        try
        {
            leftMotor = hardwareMap.dcMotor.get ("left_drive");
        }
        catch (Exception p_exeception)
        {
            m_warning_message("no left_drive");

            leftMotor = null;
        }
        try
        {
            leftFrontMotor = hardwareMap.dcMotor.get ("left_front");
        }
        catch (Exception p_exeception)
        {
            m_warning_message("no left_front");

            leftFrontMotor = null;
        }

        try
        {
            rightMotor = hardwareMap.dcMotor.get ("right_drive");
            rightMotor.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message("no right_drive");

            rightMotor = null;
        }
        try
        {
            rightFrontMotor = hardwareMap.dcMotor.get ("right_front");
            rightFrontMotor.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message("no right_front");

            rightFrontMotor = null;
        }
        waitForStart();
        sleep(10000);

     //   for(int i=0; i<2; i++) {
        rightMotor.setPower(.5);
        leftMotor.setPower(.5);
        rightFrontMotor.setPower(.5);
        leftFrontMotor.setPower(.5);
       // set_drive_power(1,1);

            sleep(1800);

        rightMotor.setPower(.5);
        leftMotor.setPower(-.5);
        rightFrontMotor.setPower(.5);
        leftFrontMotor.setPower(-.5);
            //set_drive_power(.5,-.5);

            sleep(1000);
       // }
        rightMotor.setPower(.5);
        leftMotor.setPower(.5);
        rightFrontMotor.setPower(.5);
        leftFrontMotor.setPower(.5);
        sleep(1800);

        rightMotor.setPower(0);
        leftMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);

     //   set_drive_power(0,0);

    //    if(is_touch_sensor_pressed())
    //    {
    //        set_drive_power(0,0);
    //    }

    }
}
