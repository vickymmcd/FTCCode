package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;


/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
public class APSAutoMountainLeftDelay extends PushBotHardwareSensors2 {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    //  Servo v_servo_climbertop;
    Servo v_servo_climberbottom;
    Servo v_servo_left_hand;
    Servo v_servo_left_side;
    Servo v_servo_right_side;
    Servo v_servo_left_jammer;
    Servo v_servo_flagbottom;
    Servo v_servo_flagtop;
    Servo v_servo_sensor;
    Servo v_servo_right_hand;
    Servo v_servo_right_jammer;
    ColorSensor floor_colorsensor;
    DeviceInterfaceModule cdim;
    ColorSensor wall_colorsensor;
   // ColorSensor winch_sensor;
    // TouchSensor stop_at_wall;
    UltrasonicSensor rightSensor;
    UltrasonicSensor leftSensor;
    GyroSensor zsensor;
    int sameDistance = 0;
    int state = 0;
    int close =0;
    boolean iseewhite = false;
    boolean done = false;
    boolean blueLeft;
    boolean blueRight;
    int loop = 1;

    @Override
    public void runOpMode() throws InterruptedException {


        try
        {
            leftMotor = hardwareMap.dcMotor.get ("left_drive");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("no left_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            leftMotor = null;
        }
        try
        {
            leftFrontMotor = hardwareMap.dcMotor.get ("left_front");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("no left_front");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            leftFrontMotor = null;
        }

        try
        {
            rightMotor = hardwareMap.dcMotor.get ("right_drive");
            rightMotor.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("no right_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            rightMotor = null;
        }
        try
        {
            rightFrontMotor = hardwareMap.dcMotor.get ("right_front");
            rightFrontMotor.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("no right_front");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            rightFrontMotor = null;
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
            v_servo_right_hand.setPosition (.98);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_right_hand = null;
        }
    /*    try
        {
            zsensor = hardwareMap.gyroSensor.get ("gyro");
            zsensor.calibrate();
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("gyro!");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            zsensor = null;
        }*/
        //   floor_colorsensor = hardwareMap.colorSensor.get("floor_sensor");

        //   wall_colorsensor = hardwareMap.colorSensor.get("wall_sensor");

        //  stop_at_wall = hardwareMap.touchSensor.get("wall_stopper");

        rightSensor = hardwareMap.ultrasonicSensor.get("right_optical");
        leftSensor = hardwareMap.ultrasonicSensor.get("left_optical");

        wall_colorsensor = hardwareMap.colorSensor.get("wall_sensorx42");
        wall_colorsensor.setI2cAddress(0x42);
        sleep(500);
        wall_colorsensor.enableLed(false);


        // The getI2cAddress() always returns the original address: 60 for colorSensor
        floor_colorsensor = hardwareMap.colorSensor.get("floor_sensorx3c");
        floor_colorsensor.setI2cAddress(0x3c);
        sleep(500);
        floor_colorsensor.enableLed(false);

    //    winch_sensor = hardwareMap.colorSensor.get("winch_sensorx52");
    //    winch_sensor.setI2cAddress(0x52);
    //    sleep(500);
    //    winch_sensor.enableLed(true);

        telemetry.addData("wall I2c address-2 ", wall_colorsensor.getI2cAddress());
        telemetry.addData("floor I2c address-2 ", floor_colorsensor.getI2cAddress());



        wall_colorsensor.enableLed(false);
        floor_colorsensor.enableLed(true);

        waitForStart();

        v_servo_sensor.setPosition(.14);
        sleep(10000);

        rightMotor.setPower(-.25);
        leftMotor.setPower(-.25);
        rightFrontMotor.setPower(-.25);
        leftFrontMotor.setPower(-.25);

        while (opModeIsActive()) {
            telemetry.addData("state ", state);

            switch (state) {
                case 0:

                    if (floor_colorsensor.blue() > 1 && floor_colorsensor.green() > 1 && floor_colorsensor.red() > 1) {
                        sleep(100);
                        StopRobot();
                        sleep(100);
                        state = 1;
                    }
                    break;
                case 1:
                    rightMotor.setPower(.25);
                    leftMotor.setPower(-.25);
                    rightFrontMotor.setPower(.25);
                    leftFrontMotor.setPower(-.25);
                    //sleep(50);
                    iseewhite = false;
                    while (!iseewhite) {
                        if (floor_colorsensor.blue() > 1 && floor_colorsensor.green() > 1 && floor_colorsensor.red() > 1) {
                            StopRobot();
                            sleep(100);
                            state = 2;
                            iseewhite = true;
                        }
                    }
                    break;
                case 2:
                    rightMotor.setPower(-.25);
                    leftMotor.setPower(.25);
                    rightFrontMotor.setPower(-.25);
                    leftFrontMotor.setPower(.25);
                    //  sleep(50);
                    iseewhite = false;
                    while (!iseewhite) {
                        if (floor_colorsensor.blue() > 1 && floor_colorsensor.green() > 1 && floor_colorsensor.red() > 1) {
                            StopRobot();
                            sleep(100);
                            rightMotor.setPower(-.25);
                            leftMotor.setPower(.25);
                            rightFrontMotor.setPower(-.25);
                            leftFrontMotor.setPower(.25);
                            sleep(250);
                            state = 3;
                            iseewhite = true;
                        }
                    }
                    break;
                case 3: //checks to see the distance
                    double LeftUltrasonicLevel = leftSensor.getUltrasonicLevel();
                    sleep(10);
                    double RightUltrasonicLevel = rightSensor.getUltrasonicLevel();
                    telemetry.addData("rightsensor: ", RightUltrasonicLevel);
                    telemetry.addData("leftsensor: ", LeftUltrasonicLevel);
                    telemetry.addData("sameDistance: ", sameDistance);
                    if (RightUltrasonicLevel > LeftUltrasonicLevel) {
                        sameDistance = 0;
                        rightMotor.setPower(.25);
                        leftMotor.setPower(-.25);
                        rightFrontMotor.setPower(.25);
                        leftFrontMotor.setPower(-.25);
                        sleep(20);
                        StopRobot();
                        sleep(10);
                    } else if (LeftUltrasonicLevel > RightUltrasonicLevel) {
                        sameDistance = 0;
                        rightMotor.setPower(-.25);
                        leftMotor.setPower(.25);
                        rightFrontMotor.setPower(-.25);
                        leftFrontMotor.setPower(.25);
                        sleep(20);
                        StopRobot();
                        sleep(10);
                    } else if (LeftUltrasonicLevel == RightUltrasonicLevel) {
                        sameDistance++;
                    }
                    if (sameDistance == 10) {
                        state = 4;
                    }
                    break;
                case 4: //going forward straight

                    LeftUltrasonicLevel = leftSensor.getUltrasonicLevel();
                    sleep(10);
                    RightUltrasonicLevel = rightSensor.getUltrasonicLevel();
                    telemetry.addData("rightsensor: ", RightUltrasonicLevel);
                    telemetry.addData("leftsensor: ", LeftUltrasonicLevel);
                    telemetry.addData("sameDistance: ", sameDistance);
                    if (LeftUltrasonicLevel > 18 && RightUltrasonicLevel > 18) {
                        rightMotor.setPower(-.25);
                        leftMotor.setPower(-.25);
                        rightFrontMotor.setPower(-.25);
                        leftFrontMotor.setPower(-.25);
                        sleep(20);
                        StopRobot();
                        sleep(10);
                        close = 0;
                    } else {
                        close++;
                    }
                    //    if (stop_at_wall.isPressed()) {
                    //        state = 5;
                    //   }
                    if (close > 12) {
                        state = 5;
                    }
                    break;

                case 5: //lowering arm
                    double ClimberbottomPosition = v_servo_climberbottom.getPosition();
                    while (ClimberbottomPosition > 0.3) {
                        if (ClimberbottomPosition > 0.3) {
                            v_servo_climberbottom.setPosition(ClimberbottomPosition - 0.02);
                        }
                        sleep(100);
                        ClimberbottomPosition = v_servo_climberbottom.getPosition();
                        telemetry.addData("ClimberbottomPosition: ", ClimberbottomPosition);
                    }
                    //v_servo_climberbottom.setPosition(.408);
                    //v_servo_climbertop.setPosition(.110);
                    ClimberbottomPosition = v_servo_climberbottom.getPosition();
                    while(ClimberbottomPosition < .4)
                    {
                        v_servo_climberbottom.setPosition(ClimberbottomPosition + .05);
                        ClimberbottomPosition = v_servo_climberbottom.getPosition();
                    }
                    while(ClimberbottomPosition > .25)
                    {
                        v_servo_climberbottom.setPosition(ClimberbottomPosition - .05);
                        ClimberbottomPosition = v_servo_climberbottom.getPosition();
                    }

                    sleep(500);
                    state = 7;
                    break;
           /*     case 6:
                    ClimberbottomPosition = v_servo_climberbottom.getPosition();
                    while (ClimberbottomPosition > 0.325) {
                        v_servo_climberbottom.setPosition(ClimberbottomPosition - 0.02);
                        ClimberbottomPosition = v_servo_climberbottom.getPosition();
                        sleep(100);
                    }
                    sleep(500);
                    state =7;
                    break;*/
                case 7:
                    ClimberbottomPosition = v_servo_climberbottom.getPosition();
                    while (ClimberbottomPosition < 0.9 ) {
                        if (ClimberbottomPosition < 0.9) {
                            v_servo_climberbottom.setPosition(ClimberbottomPosition + 0.1);
                        }
                        sleep(100);
                        ClimberbottomPosition = v_servo_climberbottom.getPosition();
                        telemetry.addData("ClimberbottomPosition: ", ClimberbottomPosition);
                    }

                    state = 8;
                    break;
                case 8:
                    DriveForwardTime(.25, 50);
                    if (wall_colorsensor.blue() > wall_colorsensor.red())
                    {
                        blueLeft = true;
                        blueRight = false;
                        telemetry.addData("BlueValue: ", wall_colorsensor.blue());
                        telemetry.addData("RedValue: ", wall_colorsensor.red());
                        telemetry.addData("blueLeft: ", blueLeft);
                        telemetry.addData("blueRight: ", blueRight);
                    }
                    else
                    {
                        blueLeft = false;
                        blueRight = true;
                        telemetry.addData("BlueValue: ", wall_colorsensor.blue());
                        telemetry.addData("RedValue: ", wall_colorsensor.red());
                        telemetry.addData("blueLeft: ", blueLeft);
                        telemetry.addData("blueRight: ", blueRight);
                    }
                    telemetry.addData("blueLeft: ", blueLeft);
                    telemetry.addData("blueRight: ", blueRight);
                  //  sleep(10000);
                    if (blueLeft) {
                        DriveForwardTime(.25, 500);
                        TurnRightTime(.25, 300);
                        DriveBackwardTime(.25, 600);
                        v_servo_sensor.setPosition(.39);
                        sameDistance = 0;
                        while (sameDistance < 10) {
                            LeftUltrasonicLevel = leftSensor.getUltrasonicLevel();
                            sleep(10);
                            RightUltrasonicLevel = rightSensor.getUltrasonicLevel();
                            telemetry.addData("rightsensor: ", RightUltrasonicLevel);
                            telemetry.addData("leftsensor: ", LeftUltrasonicLevel);
                            telemetry.addData("sameDistance: ", sameDistance);
                            if (RightUltrasonicLevel > LeftUltrasonicLevel) {
                                sameDistance = 0;
                                rightMotor.setPower(.25);
                                leftMotor.setPower(-.25);
                                rightFrontMotor.setPower(.25);
                                leftFrontMotor.setPower(-.25);
                                sleep(20);
                                StopRobot();
                                sleep(10);
                            } else if (LeftUltrasonicLevel > RightUltrasonicLevel) {
                                sameDistance = 0;
                                rightMotor.setPower(-.25);
                                leftMotor.setPower(.25);
                                rightFrontMotor.setPower(-.25);
                                leftFrontMotor.setPower(.25);
                                sleep(20);
                                StopRobot();
                                sleep(10);
                            } else if (LeftUltrasonicLevel == RightUltrasonicLevel) {
                                sameDistance++;
                            }
                            if (sameDistance == 10) {
                                DriveBackwardTime(.25, 300);
                            }
                        }
                    }

                    else if (blueRight) {
                        v_servo_sensor.setPosition(.39);
                        DriveBackwardTime(.25, 300);

                    }


                    state = 9;
                    break;



                default:
                    telemetry.addData("rightsensor: ", rightSensor.getUltrasonicLevel());
                    telemetry.addData("leftsensor: ", leftSensor.getUltrasonicLevel());
                    telemetry.addData("sameDistance: ", sameDistance);
                 //   telemetry.addData("gyro value: ", zsensor.getRotation());
                    StopRobot();
            }
            telemetry.addData("floor aClear ", floor_colorsensor.alpha());
            telemetry.addData("floor aRed   ", floor_colorsensor.red());
            telemetry.addData("floor aGreen ", floor_colorsensor.green());
            telemetry.addData("floor aBlue  ", floor_colorsensor.blue());
            telemetry.addData("wall aClear ", wall_colorsensor.alpha());
            telemetry.addData("wall aRed   ", wall_colorsensor.red());
            telemetry.addData("wall aGreen ", wall_colorsensor.green());
            telemetry.addData("wall aBlue  ", wall_colorsensor.blue());
            //     telemetry.addData("Touch sensor=", stop_at_wall.isPressed());
        }


        //  if(!white)
        //  {
        //      white = true;
        //      rightMotor.setPower(.25);
        //      leftMotor.setPower(.25);
        //     rightFrontMotor.setPower(.25);
        //      leftFrontMotor.setPower(.25);
        //     sleep(10);
        //  }







        // wall_colorsensor.enableLed(false);
        telemetry.addData("floor aClear ", floor_colorsensor.alpha());
        telemetry.addData("floor aRed   ", floor_colorsensor.red());
        telemetry.addData("floor aGreen ", floor_colorsensor.green());
        telemetry.addData("floor aBlue  ", floor_colorsensor.blue());
        //  telemetry.addData("wall aClear ", wall_colorsensor.alpha());
        //  telemetry.addData("wall aRed   ", wall_colorsensor.red());
        //  telemetry.addData("wall aGreen ", wall_colorsensor.green());
        //   telemetry.addData("wall aBlue  ", wall_colorsensor.blue());
        //      telemetry.addData("Touch sensor=", stop_at_wall.isPressed());
    }




    public void DriveForwardTime(double power,long time) throws InterruptedException
    {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftFrontMotor.setPower(power);
        sleep(time);
    }
    public void DriveBackwardTime(double power,long time) throws InterruptedException
    {
        rightMotor.setPower(-power);
        leftMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
        leftFrontMotor.setPower(-power);
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