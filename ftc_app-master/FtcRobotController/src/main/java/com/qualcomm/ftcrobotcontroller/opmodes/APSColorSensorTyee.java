package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
public class APSColorSensorTyee extends PushBotHardwareSensors2 {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    //TyeeModernRoboticsColorSensor floor_colorsensor;
    //TyeeModernRoboticsColorSensor wall_colorsensor;
    DeviceInterfaceModule dim;

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

        dim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        ColorSensor wall_colorsensor = new TyeeModernRoboticsColorSensorNew(dim, 3) {
            @Override
            public void setI2cAddress(int i) {
                this.setNewI2CAddress(0x42);
            }

            @Override
            public int getI2cAddress() {
                return 0;
            }
        };
        //ColorSensor wall_colorsensor = new TyeeModernRoboticsColorSensor(dim, 3);
        //OLD: wall_colorsensor = hardwareMap.colorSensor.get("wall_sensorx42");
        wall_colorsensor.setI2cAddress(0x42); //= TyeeModernRoboticsColorSensorx42_3("")
        //wall_colorsensor.portIsReady(3);

        //hardwareMap.colorSensor.get("wall_sensorx42");
        // The getI2cAddress() always returns the original address: 60 for colorSensor
        //telemetry.addData("wall I2c address ", wall_colorsensor.getI2cAddress());
        //wall_colorsensor.setI2cAddress(0x42);
        //wall_colorsensor.enableLed(false);

        //ColorSensor floor_colorsensor = new TyeeModernRoboticsColorSensorX52(dim, 0);
        // OLD: floor_colorsensor = hardwareMap.colorSensor.get("floor_sensorx52");
        // The getI2cAddress() always returns the original address: 60 for colorSensor
        //telemetry.addData("floor I2c address ", floor_colorsensor.getI2cAddress());
        //floor_colorsensor.setI2cAddress(0x52);
        //floor_colorsensor.enableLed(false);


        //sleep(500);
        //floor_colorsensor.enableLed(true);
        waitForStart();

        int i = 5;
        while (i>0)
        {
            sleep(500);
            wall_colorsensor.enableLed(true);
            //floor_colorsensor.enableLed(false);
            sleep(500);
            wall_colorsensor.enableLed(false);
            //floor_colorsensor.enableLed(true);
            i--;
        }
        wall_colorsensor.enableLed(true);
        //floor_colorsensor.enableLed(true);

        while(opModeIsActive()){

            //floor_colorsensor.enableLed(true);
            //wall_colorsensor.enableLed(false);
            //telemetry.addData("floor aClear ", floor_colorsensor.alpha());
            //telemetry.addData("floor aRed   ", floor_colorsensor.red());
            //telemetry.addData("floor aGreen ", floor_colorsensor.green());
            //telemetry.addData("floor aBlue  ", floor_colorsensor.blue());
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
