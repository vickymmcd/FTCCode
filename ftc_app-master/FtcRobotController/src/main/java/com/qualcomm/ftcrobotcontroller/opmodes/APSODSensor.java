package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
public class APSODSensor extends PushBotHardwareSensors2 {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    ColorSensor floor_colorsensor;
    ColorSensor wall_colorsensor;
    TouchSensor stop_at_wall;
    UltrasonicSensor rightSensor;
    UltrasonicSensor leftSensor;
    int state = 0;
    boolean iseewhite = false;
    boolean done = false;
    int loop = 1;

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
        floor_colorsensor = hardwareMap.colorSensor.get("floor_sensor");

        wall_colorsensor = hardwareMap.colorSensor.get("wall_sensor");

        stop_at_wall = hardwareMap.touchSensor.get("wall_stopper");

        rightSensor = hardwareMap.ultrasonicSensor.get("right_optical");
        leftSensor = hardwareMap.ultrasonicSensor.get("left_optical");

        floor_colorsensor.enableLed(false);

        waitForStart();

        floor_colorsensor.enableLed(true);

        while(opModeIsActive())
        {

            telemetry.addData("rightsensor: ", rightSensor.getUltrasonicLevel());
            telemetry.addData("leftsensor: ", leftSensor.getUltrasonicLevel());


            //  if(!white)
            //  {
            //      white = true;
            //      rightMotor.setPower(.25);
            //      leftMotor.setPower(.25);
            //     rightFrontMotor.setPower(.25);
            //      leftFrontMotor.setPower(.25);
            //     sleep(10);
            //  }


        }




        // wall_colorsensor.enableLed(false);
        telemetry.addData("floor aClear ", floor_colorsensor.alpha());
        telemetry.addData("floor aRed   ", floor_colorsensor.red());
        telemetry.addData("floor aGreen ", floor_colorsensor.green());
        telemetry.addData("floor aBlue  ", floor_colorsensor.blue());
        telemetry.addData("wall aClear ", wall_colorsensor.alpha());
        telemetry.addData("wall aRed   ", wall_colorsensor.red());
        telemetry.addData("wall aGreen ", wall_colorsensor.green());
        telemetry.addData("wall aBlue  ", wall_colorsensor.blue());
        telemetry.addData("Touch sensor=", stop_at_wall.isPressed());
    }




    public void DriveForwardTime(double power,long time) throws InterruptedException
    {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftFrontMotor.setPower(power);
        sleep(time);
    }
    public void TurnRightTime(double power,long time) throws InterruptedException
    {
        rightMotor.setPower(-power);
        leftMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftFrontMotor.setPower(power);
        sleep(time);
    }
    public void TurnLeftTime(double power,long time) throws InterruptedException
    {
        rightMotor.setPower(power);
        leftMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftFrontMotor.setPower(-power);
        sleep(time);
    }
    public void StopRobot()
    {
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
    }
}