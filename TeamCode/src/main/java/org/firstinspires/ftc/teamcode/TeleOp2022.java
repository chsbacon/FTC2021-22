/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;
import java.util.Locale;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//Graham Branch
@TeleOp(name="TeleOp 2022", group="Linear Opmode")
//@Disabled
public class TeleOp2022 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap2022 robot = new HardwareMap2022();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        double x;
        double y;
        double r;
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        double fastSlow;

        //start Orientation will always be 0; this is the heading when initialized
        Orientation startOrientation;
        startOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation currentOrientation;


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //grabs current orientation for this iteration of opModeIsActive
            currentOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            if(gamepad1.a){
                fastSlow = 2;
            }
            else{
                fastSlow = 1;
            }

            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            r = gamepad1.right_stick_x;


            // do not let rotation dominate movement
            r = r / 2;

            // calculate the power for each wheel
            frontLeft = +y - x + r;
            backLeft = +y + x + r;

            frontRight = -y - x + r;
            backRight = -y + x + r;



            robot.frontLeftMotor.setPower(frontLeft/fastSlow);
            robot.frontRightMotor.setPower(frontRight/fastSlow);
            robot.backLeftMotor.setPower(backLeft/fastSlow);
            robot.backRightMotor.setPower(backRight/fastSlow);









            //telemtry for motors
            telemetry.addData("front left", "%.2f", frontLeft/fastSlow);
            telemetry.addData("front right", "%.2f", frontRight/fastSlow);
            telemetry.addData("back left", "%.2f", backLeft/fastSlow);
            telemetry.addData("back right", "%.2f", backRight/fastSlow);

            //telemetry for IMU
            telemetry.addData("startOrientation", formatAngle(startOrientation.angleUnit, startOrientation.firstAngle));
            telemetry.addData("currentOrientation", formatAngle(currentOrientation.angleUnit, currentOrientation.firstAngle));


            telemetry.update();

        }
    }





    //just formatting stuff for the angles -- this was copied and pasted
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }


    //just formatting stuff for the degrees -- this was copied and pasted
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



    void rotateToHeadingV1(double pwr, Orientation target){

        double kP = 0;
        double kI = 0;
        double kD = 0;

        // set to a big number so it doesn't accidently match the target angle
        //therefore hypothetically completing the while-loop accidently
        double currAng = 10000;

        Orientation targetOrient;
        Orientation currOrient;

        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);

        double integralSum = 0;
        double lastError = 0;
        double error;
        double derivative;
        double out;

        ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



        while( currAng != targAng ){

            pidTimer.reset();

            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            error = targAng - currAng;

            derivative = (error - lastError) / pidTimer.milliseconds();

            integralSum = integralSum + (error * pidTimer.time());

            out = (kP * error) + (kI * integralSum) + (kD * derivative);
            //Still have to see if out is -1 < out < 1

            robot.frontLeftMotor.setPower(pwr + out);
            robot.frontRightMotor.setPower(pwr - out);
            robot.backLeftMotor.setPower(pwr + out);
            robot.backRightMotor.setPower(pwr - out);

            lastError = error;

        }
    }


}
