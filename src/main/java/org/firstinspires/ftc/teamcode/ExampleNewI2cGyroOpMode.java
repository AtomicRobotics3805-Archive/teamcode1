package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.I2cAddr;
        import com.qualcomm.robotcore.hardware.I2cDevice;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl ;

/**
 * This is an example OpMode that shows how to read the Modern Robotics gyro sensor
 * using I2cDeviceSynch calls in the new FTC app beta.
 */


@Autonomous(name = "Sensor: MRGyro", group = "Sensor")
//@Disabled
public class ExampleNewI2cGyroOpMode extends OpMode {
    int heading = 0; //Variable for Heading data
    int intZValue = 0; //Variable for the integrated Z value

    byte[] gyro1Cache; //The read will return an array of bytes. They are stored in this variable

    public static final byte GYRO1ADDRESS = 0x20; //Default I2C address for MR gyro
    public static final int GYRO1_REG_START = 0x04; //Register to start reading
    public static final int GYRO1_READ_LENGTH = 14; //Number of byte to read
    public static final int GYRO_COMMAND_REGISTER = 0x03; //Register to issue commands to the gyro
    public static final byte GYRO_CALIBRATE_COMMAND = 0x4E; //Command to calibrate the gyro

    public I2cDevice gyro1;
    public I2cDeviceSynch gyro1Reader;
    public I2cAddr gyro1Addr;

    public ExampleNewI2cGyroOpMode() {
    }

    @Override
    public void init() {
        gyro1Addr = I2cAddr.create8bit(GYRO1ADDRESS);
        gyro1 = hardwareMap.i2cDevice.get("gyro1MR");
        gyro1Reader = new I2cDeviceSynchImpl(gyro1, gyro1Addr, false);
        gyro1Reader.engage();
    }

    @Override
    public void init_loop() {
//Calibrate the gyro if it is not reading 0
        if (gyro1Reader.read8(4) != 0) {
            gyro1Reader.write8(GYRO_COMMAND_REGISTER, GYRO_CALIBRATE_COMMAND);
        }
    }


    @Override
    public void loop() {

        gyro1Cache = gyro1Reader.read(GYRO1_REG_START, GYRO1_READ_LENGTH);

//One byte only goes to 256 values. Two bytes must be combined to get 360 degrees
        heading = 0X000000FF & (int)gyro1Cache[1];
        heading = (heading << 8 | (0X000000FF & (int)gyro1Cache[0]));

        intZValue = 0X000000FF & (int)gyro1Cache[3];
        intZValue = (intZValue << 8 | (0X000000FF & (int)gyro1Cache[2]));

// send the info back to driver station using telemetry function.

        telemetry.addData("Gyro Heading1", gyro1Cache[0]);
        telemetry.addData("Gyro Heading2", gyro1Cache[1]);
        telemetry.addData("Gyro HeadingTot", heading );
        telemetry.addData("Gyro Int Z1", gyro1Cache[2]);
        telemetry.addData("Gyro Int Z2", gyro1Cache[3]);
        telemetry.addData("Gyro Int ZTot", (short) intZValue);

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