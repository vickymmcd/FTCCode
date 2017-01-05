package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
public class APSColorSensorMatthew extends PushBotHardwareSensors2 {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    ColorSensor floor_colorsensor;
    DeviceInterfaceModule cdim;



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
      //  try {
      //      floor_colorsensor = hardwareMap.colorSensor.get("floor_sensor");
      //  }
     //   catch(Exception p_exception)
      //  {
      //      m_warning_message("no floor sensor");
      //      floor_colorsensor = null;
       // }

        try{
            cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        }
        catch(Exception p_exception)
        {
            m_warning_message("can't find device interface module");
            cdim = null;
        }
        ColorSensor wall_colorsensor;
        /**** IMPORTANT NOTE ******/
        // You need to add a line like this at the top of your op mode
        // to update the I2cAddress in the driver.
        //irSeeker.setI2cAddress(newAddress);
        /***************************/

        if (cdim != null){
            wall_colorsensor = new TyeeModernRoboticsColorSensor(cdim, 3);
            wall_colorsensor.setI2cAddress(0x42);
//
        }
        else
        {
            wall_colorsensor = null;
            m_warning_message("no wall sensor");
        }



       // floor_colorsensor.enableLed(false);
        wall_colorsensor.enableLed(true);
        sleep(500);



        waitForStart();
        while(opModeIsActive()){


            if (wall_colorsensor==null)
            {
                telemetry.addData("alert", "something is equal to null");
            }
            else if(wall_colorsensor!= null) {
              //  floor_colorsensor.enableLed(true);
                wall_colorsensor.enableLed(false);
             //   telemetry.addData("floor aClear ", floor_colorsensor.alpha());
             //   telemetry.addData("floor aRed   ", floor_colorsensor.red());
             //   telemetry.addData("floor aGreen ", floor_colorsensor.green());
             //   telemetry.addData("floor aBlue  ", floor_colorsensor.blue());
                telemetry.addData("wall aClear ", wall_colorsensor.alpha());
                telemetry.addData("wall aRed   ", wall_colorsensor.red());
                telemetry.addData("wall aGreen ", wall_colorsensor.green());
                telemetry.addData("wall aBlue  ", wall_colorsensor.blue());
              //  update_telemetry();
            }


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
