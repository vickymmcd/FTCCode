package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.concurrent.locks.Lock;

/**
 * This is a custom made program by Team 9915 to change the Modern Robotics Color Sensor I2C Address.
 * <p>MR Color Sensor default address: 0x3c
 * If you want to use multiple MR color sensors, you need to change the I2C address.
 * <p>
 *
 * In this program, the I2C address is changed to 0x??, you can use this program to change additional ones.
 *
 * @author: Team 9915
 *
 */
public class MrKazmerI2C_AddressChange extends LinearOpMode {

    public static final int ADDRESS_SET_NEW_I2C_ADDRESS = 0x52;
    public static final byte COLOR_SENSOR_ORIGINAL_ADDRESS = 0x42;

    // Expected bytes from the Modern Robotics Color Sensor memory map
    public static final byte MANUFACTURER_CODE = 0x4d;
    public static final byte COLOR_SENSOR_FIRMWARE_REV = 0x10;
    public static final byte COLOR_SENSOR_SENSOR_ID = 0x43;

    // trigger bytes used to change I2C address on ModernRobotics sensors.
    public static final byte TRIGGER_BYTE_1 = 0x55;
    public static final byte TRIGGER_BYTE_2 = (byte) 0xaa;

    public static final int READ_MODE = 0x80;
    public static final int ADDRESS_MEMORY_START = 0x03;
    public static final int TOTAL_MEMORY_LENGTH = 0x06;

    public static final int BUFFER_CHANGE_ADDRESS_LENGTH = 0x03;

    int port = 0;

    byte[] readCache;
    Lock readLock;
    byte[] writeCache;
    Lock writeLock;

    int currentAddress = COLOR_SENSOR_ORIGINAL_ADDRESS;
    // I2c addresses on Modern Robotics devices must be divisible by 2, and between 0x7e and 0x10
    // Different hardware may have different rules.
    // Be sure to read the requirements for the hardware you're using!
    int newAddress = ADDRESS_SET_NEW_I2C_ADDRESS;

    DeviceInterfaceModule dim;

    @Override
    public void runOpMode() throws InterruptedException {

        // set up the hardware devices we are going to use
        dim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        readCache = dim.getI2cReadCache(port);
        readLock = dim.getI2cReadCacheLock(port);
        writeCache = dim.getI2cWriteCache(port);
        writeLock = dim.getI2cWriteCacheLock(port);

        telemetry.addData("!!! COLOR Sensors ONLY !!!","");
        telemetry.addData("Sensor must be in I2c port: ", port);
        telemetry.addData("     initial address: ", currentAddress);
        telemetry.addData(" writing new address: ", newAddress);
        telemetry.addData("STOP now if these are incorrect!","");

        // wait for the start button to be pressed
        waitForStart();

        performAction("read", port, currentAddress, ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH);

        while(!dim.isI2cPortReady(port)) {
            telemetry.addData("I2cAddressChange", "waiting for the port to be ready...");
            sleep(1000);
        }

        // update the local cache
        dim.readI2cCacheFromController(port);

        // make sure the first bytes are what we think they should be.
        // Expected bytes from the Modern Robotics Color Sensor memory map
        int count = 0;
        int[] initialArray = {READ_MODE, currentAddress, ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH, COLOR_SENSOR_FIRMWARE_REV, MANUFACTURER_CODE, COLOR_SENSOR_SENSOR_ID};
        while (!foundExpectedBytes(initialArray, readLock, readCache)) {
            telemetry.addData("I2cAddressChange", "Confirming that we're reading the correct bytes...");
            dim.readI2cCacheFromController(port);
            sleep(1000);
            count++;
            // if we go too long with failure, we probably are expecting the wrong bytes.
            if (count >= 10)  {
                telemetry.addData("I2cAddressChange", String.format("Looping too long with no change, probably have the wrong address. Current address: 0x%02x", currentAddress));
                //hardwareMap.irSeekerSensor.get(String.format("Looping too long with no change, probably have the wrong address. Current address: 0x%02x", currentAddress));
                telemetry.addData("Press STOP...","");
                sleep(10000);
            }
        }

        // Enable writes to the correct segment of the memory map.
        performAction("write", port, currentAddress, ADDRESS_SET_NEW_I2C_ADDRESS, BUFFER_CHANGE_ADDRESS_LENGTH);

        waitOneFullHardwareCycle();

        // Write out the trigger bytes, and the new desired address.
        writeNewAddress();
        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);

        telemetry.addData("I2cAddressChange", "Giving the hardware some time to make the change...");

        // Changing the I2C address takes some time.
        for (int i = 0; i < 5000; i++) {
            waitOneFullHardwareCycle();
        }

        // Query the new address and see if we can get the bytes we expect.
        dim.enableI2cReadMode(port, newAddress, ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH);
        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);

        count=0;
        int[] confirmArray = {READ_MODE, newAddress, ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH, COLOR_SENSOR_FIRMWARE_REV, MANUFACTURER_CODE, COLOR_SENSOR_SENSOR_ID};
        while (!foundExpectedBytes(confirmArray, readLock, readCache)) {
            telemetry.addData("I2cAddressChange", "Have not confirmed the changes yet...");
            dim.readI2cCacheFromController(port);
            sleep(1000);
            count++;
            // if we go too long with failure, we probably are expecting the wrong bytes.
            if (count >= 10)  {
                telemetry.addData("I2cAddressChange", String.format("Looping too long with no change, something is wrong. Current address: 0x%02x", currentAddress));
                //hardwareMap.irSeekerSensor.get(String.format("Looping too long with no change, probably have the wrong address. Current address: 0x%02x", currentAddress));
                telemetry.addData("FAILED...","");
                break;
            }
        }

        telemetry.addData("I2cAddressChange", "Successfully changed the I2C address." + String.format("New address: %02x", newAddress));
        telemetry.addData("!!! Mark COLOR Sensor with this value: ",newAddress);
    }

    private boolean foundExpectedBytes(int[] byteArray, Lock lock, byte[] cache) {
        try {
            lock.lock();
            boolean allMatch = true;
            StringBuilder s = new StringBuilder(300 * 4);
            String mismatch = "";
            for (int i = 0; i < byteArray.length; i++) {
                s.append(String.format("expected: %02x, got: %02x \n", TypeConversion.unsignedByteToInt((byte) byteArray[i]), cache[i]));
                if (TypeConversion.unsignedByteToInt(cache[i]) != TypeConversion.unsignedByteToInt( (byte) byteArray[i])) {
                    mismatch = String.format("i: %d, byteArray[i]: %02x, cache[i]: %02x", i, byteArray[i], cache[i]);
                    allMatch = false;
                }
            }
            RobotLog.e(s.toString() + "\n allMatch: " + allMatch + ", mismatch: " + mismatch);
            return allMatch;
        } finally {
            lock.unlock();
        }
    }

    private void performAction(String actionName, int port, int i2cAddress, int memAddress, int memLength) {
        if (actionName.equalsIgnoreCase("read")) dim.enableI2cReadMode(port, i2cAddress, memAddress, memLength);
        if (actionName.equalsIgnoreCase("write")) dim.enableI2cWriteMode(port, i2cAddress, memAddress, memLength);

        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);
        dim.readI2cCacheFromController(port);
    }

    private void writeNewAddress() {
        try {
            writeLock.lock();
            writeCache[4] = (byte) newAddress;
            writeCache[5] = TRIGGER_BYTE_1;
            writeCache[6] = TRIGGER_BYTE_2;
        } finally {
            writeLock.unlock();
        }
    }
}