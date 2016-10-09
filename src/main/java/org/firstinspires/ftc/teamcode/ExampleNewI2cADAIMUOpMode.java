package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

// Change only made in comment to test git

/**
 * This is an example OpMode that shows how to read the Modern Robotics gyro sensor
 * using I2cDeviceSynch calls in the new FTC app beta.
 */
@Autonomous(name = "Sensor: AdafruitIMU", group = "Sensor")
//@Disabled
public class ExampleNewI2cADAIMUOpMode extends OpMode {
    int heading = 0; //Variable for Heading data
    int intZValue = 0; //Variable for the integrated Z value

    byte[] gyro1Cache; //The read will return an array of bytes. They are stored in this variable

    public static final byte GYRO1ADDRESS = 0x50; //Default I2C address for MR gyro
    public static final int GYRO1_REG_START = 0x04; //Register to start reading
    public static final int GYRO1_READ_LENGTH = 14; //Number of byte to read
    public static final int GYRO_COMMAND_REGISTER = 0x03; //Register to issue commands to the gyro
    public static final byte GYRO_CALIBRATE_COMMAND = 0x4E; //Command to calibrate the gyro
I2cAddr gyroADAAddr;

    public static final byte BNO055_PAGE_ID_ADDR = 0x07;
    public static final byte BNO055_SYS_TRIGGER_ADDR = 0x3F;
    public static final byte OPERATION_MODE_IMU  = 0X08;
    public static final byte  BNO055_OPR_MODE_ADDR = 0x3D;
    public static final byte EUL_DATA_X_LSB = 0x1A;
    public static final byte QUA_DATA_X_LSB = 0x22;


    public I2cDevice gyroADA;
    public I2cDeviceSynch gyro1Reader;

    private void snooze(long milliSecs){//Simple utility for sleeping (thereby releasing the CPU to
        // threads other than this one)
        try {
            Thread.sleep(milliSecs);
        } catch (InterruptedException e){}
    }

    public ExampleNewI2cADAIMUOpMode() {
    }
    @Override
    public void init() {
        gyroADAAddr = I2cAddr.create8bit(GYRO1ADDRESS);
        gyroADA = hardwareMap.i2cDevice.get("bno055");
        gyro1Reader = new I2cDeviceSynchImpl(gyroADA, gyroADAAddr, false);
        gyro1Reader.engage();
        //Sets the PAGE_ID bit for page 0 (Table 4-2)
        gyro1Reader.write8(BNO055_PAGE_ID_ADDR, 0x00);
        //The "E" sets the RST_SYS and RST_INT bits, and sets the CLK_SEL bit
        //, to select the external IMU clock mounted on the Adafruit board (Table 4-2, and p.70). In
        // the lower 4 bits, a "1" sets the commanded Self Test bit, which causes self-test to run (p. 46)
        gyro1Reader.write8(BNO055_SYS_TRIGGER_ADDR, 0xE1);

        snooze(1000);

        gyro1Reader.write8( BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMU);



    }

    @Override
    public void init_loop() {
        gyro1Cache = gyro1Reader.read(BNO055_OPR_MODE_ADDR, 1);
        telemetry.addData("1", "Mode " + gyro1Cache[0]);

    }


    @Override
    public void loop() {

        gyro1Cache = gyro1Reader.read(EUL_DATA_X_LSB, 2);

//One byte only goes to 256 values. Two bytes must be combined to get 360 degrees
        heading = 0X000000FF & (int)gyro1Cache[1];
        heading = (heading << 8 | (0X000000FF & (int)gyro1Cache[0]));
        heading = heading /16;

        //intZValue = 0X000000FF & (int)gyro1Cache[3];
        //intZValue = (intZValue << 8 | (0X000000FF & (int)gyro1Cache[2]));

// send the info back to driver station using telemetry function.

        telemetry.addData("Gyro Heading LSB", gyro1Cache[0]);
        telemetry.addData("Gyro Heading MSB", gyro1Cache[1]);
        telemetry.addData("Gyro HeadingTot", heading );
        //telemetry.addData("Gyro Int Z1", gyro1Cache[2]);
        //telemetry.addData("Gyro Int Z2", gyro1Cache[3]);
        //telemetry.addData("Gyro Int ZTot", (short) intZValue);

    }


    @Override
    public void stop() {

    }
/*
I2C Registers
Address Function
0x00 Sensor Firmware Revision
0x01 Manufacturer Code
0x02 Sensor ID Code
0x03 Command
0x04/0x05 Heading Data (lsb:msb)
0x06/0x07 Integrated Z Value (lsb:msb)
0x08/0x09 Raw X Value (lsb:msb)
0x0A/0x0B Raw Y Value (lsb:msb)
0x0C/0x0D Raw Z Value (lsb:msb)
0x0E/0x0F Z Axis Offset (lsb:msb)
0x10/0x11 Z Axis Scaling Coefficient (lsb:msb)

Commands
Command	Operation
0x00	Normal measurement mode
0x4E	Null gyro offset and reset Z axis integrator
0x52	Reset Z axis integrator
0x57	Write EEPROM data
*/
}