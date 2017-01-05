package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
public class APSColorSensor extends PushBotHardwareSensors2 {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    ColorSensor floor_colorsensor;
    ColorSensor wall_colorsensor;

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

        wall_colorsensor = hardwareMap.colorSensor.get("wall_sensorx42");
        wall_colorsensor.setI2cAddress(0x42);
        sleep(500);
        wall_colorsensor.enableLed(false);


        // The getI2cAddress() always returns the original address: 60 for colorSensor
        floor_colorsensor = hardwareMap.colorSensor.get("floor_sensorx52");
        floor_colorsensor.setI2cAddress(0x52);
        sleep(500);
        floor_colorsensor.enableLed(false);

        telemetry.addData("wall I2c address-2 ", wall_colorsensor.getI2cAddress());
        telemetry.addData("floor I2c address-2 ", floor_colorsensor.getI2cAddress());

        waitForStart();

        int i = 5;
        while (i>0)
        {
            sleep(500);
            wall_colorsensor.enableLed(true);
            floor_colorsensor.enableLed(false);
            sleep(500);
            wall_colorsensor.enableLed(false);
            floor_colorsensor.enableLed(true);
            i--;
        }
        wall_colorsensor.enableLed(true);
        floor_colorsensor.enableLed(true);


        while(opModeIsActive()){

            telemetry.addData("floor aClear ", floor_colorsensor.alpha());
            telemetry.addData("floor aRed   ", floor_colorsensor.red());
            telemetry.addData("floor aGreen ", floor_colorsensor.green());
            telemetry.addData("floor aBlue  ", floor_colorsensor.blue());
            telemetry.addData("wall aClear ", wall_colorsensor.alpha());
            telemetry.addData("wall aRed   ", wall_colorsensor.red());
            telemetry.addData("wall aGreen ", wall_colorsensor.green());
            telemetry.addData("wall aBlue  ", wall_colorsensor.blue());
        }
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
