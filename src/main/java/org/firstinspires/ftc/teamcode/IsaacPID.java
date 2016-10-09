/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.text.SimpleDateFormat;
import java.util.Date;

import dalvik.annotation.TestTarget;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
@Autonomous(name = "Sensor: PID", group = "Sensor")
//@Disabled
public class IsaacPID extends OpMode {

    //AdafruitIMU Gyro;

    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    DcMotor LeftMotor;
    DcMotor RightMotor;
    double error;
    double integral = 0;
    double derivative = 0;
    double Ke = 0.001;
    double Ki = 0;
    double Kd = 0;
    double lastError;
    double output;
    double previousTime = 0;
    boolean isFirstLoop = true;

    @Override
    public void init() {

        /*try {
            Gyro = new AdafruitIMU(hardwareMap, "bno055"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte) AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e) {
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
        }*/


        LeftMotor = hardwareMap.dcMotor.get("LM");
        //RightMotor = hardwareMap.dcMotor.get("RM");

        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void start() {

        //Gyro.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.

    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {

    }

    public double updateTime() {
        double deltaTime;
        double currentTime = (double) System.nanoTime() / 1000000000.0;
        if (previousTime == 0){
            deltaTime = 0;
        } else {
            deltaTime = currentTime - previousTime;
        }
        previousTime = currentTime;

        return deltaTime;
    }

    public double loopPID(double actualValue, double setpoint) {

        double deltaTime = updateTime();

        error = actualValue - setpoint;

        output = (Ke*error);

        if(isFirstLoop) {
            isFirstLoop = false;
        } else {
            integral = integral + (error * deltaTime);
            derivative = error - lastError;
            output += (Ki*integral)+(Kd*derivative);
        }

        lastError = error;

        return output;

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        //Gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

        double output = loopPID(-LeftMotor.getCurrentPosition(), 999);

        LeftMotor.setPower(output);
        //RightMotor.setPower(output);

        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
        telemetry.addData("1", "ENC " + LeftMotor.getCurrentPosition());
        telemetry.addData("3", "power " + output);



    }
}