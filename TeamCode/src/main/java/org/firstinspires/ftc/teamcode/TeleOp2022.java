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
        double fastSlow = 1;

        int linearSlideTicks = 0;

        //start Orientation will always be 0; this is the heading when robot is initialized
        Orientation startOrientation;
        startOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation currentOrientation;


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // to grab heading from robot
            //robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            //grabs current orientation for this iteration of opModeIsActive
            currentOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //fast slow + auto test
            /*
            if(gamepad1.a){
                fastSlow = 2;
            }
            else{
                fastSlow = 1;
            }



            //auto test
            /*if(gamepad1.b){
                //distance from wall to shipping hub is 550
                robot.driveForwardUseBackwardDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),550);
                //rotate towards shipping hub
                robot.rotateToHeading(0,55);
                //drive slow straight for 1/4 a second
                robot.driveForwardUseTime(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
                //place
                robot.spinCarouselMotor();
                //drive slow backwards for 1/4 a second
                robot.driveBackwardUseTime(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
                //rotate towards to -90
                robot.rotateToHeading(0,-90);
                robot.driveForwardUseFrontDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),300);
                robot.rotateToHeading(0,0);
                robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
                robot.driveBackwardUseBackDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),330);
                robot.spinCarouselMotor();
                robot.driveForwardUseBackwardDistance(.25,startOrientation,1200);
                robot.driveForwardUseTime(.25,startOrientation,1000);
                robot.rotateToHeading(0,90);
                robot.driveForwardUseTime(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),5500);
                robot.rotateToHeading(0,0);
                robot.driveBackwardUseBackDistance(.25,startOrientation,650);
                robot.rotateToHeading(0,90);
                robot.driveForwardUseTime(.9,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),1250);

            }
            */













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
            //telemetry.addData("front left", "%.2f", frontLeft/fastSlow);
            //telemetry.addData("front right", "%.2f", frontRight/fastSlow);
            //telemetry.addData("back left", "%.2f", backLeft/fastSlow);
            //telemetry.addData("back right", "%.2f", backRight/fastSlow);
            //telemetry for IMU
            //telemetry.addData("startOrientation", formatAngle(startOrientation.angleUnit, startOrientation.firstAngle));
            telemetry.addData("currentOrientation", formatAngle(currentOrientation.angleUnit, currentOrientation.firstAngle));
            //telemtry for Distance Sensors
            telemetry.addData("Front Sensor", String.format("%.01f mm", robot.frontDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("Right Sensor", String.format("%.01f mm", robot.rightDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("Back Sensor", String.format("%.01f mm", robot.backDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("Left Sensor", String.format("%.01f mm", robot.leftDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("current Angle",currentOrientation);

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


    public void moveLinearSlide(int myTicks){
        //robot.leftLinearSlideMotor.setTargetPosition(myTicks);
        robot.rightLinearSlideMotor.setTargetPosition(myTicks);

        //robot.leftLinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightLinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.leftLinearSlideMotor.setPower(0.1);
        robot.rightLinearSlideMotor.setPower(0.1);

        while(/*robot.leftLinearSlideMotor.isBusy() && */robot.rightLinearSlideMotor.isBusy()){
        }
        //robot.leftLinearSlideMotor.setPower(0);
        robot.rightLinearSlideMotor.setPower(0);

        //robot.leftLinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

//auto test
/*
                robot.driveBackwardUseBackDistance(.25,startOrientation,500);
                robot.strafeRight(.5,startOrientation,1500);
                robot.rotateToHeading(0,-90);
                robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),1500);
                robot.driveBackwardUseBackDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),330);
                robot.spinCarouselMotor();
                robot.driveForwardUseBackwardDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),800);
                robot.strafeLeft(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 750 );
                robot.rotateToHeading(0,0);
                robot.driveForwardUseTime(.25,startOrientation,4000);
                robot.driveForwardUseTime(.7,startOrientation,3000);
 */

//Basic Blue Duck Side Auto
/*
                robot.driveForwardUseBackwardDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),550);
                //rotate towards shipping hub
                robot.rotateToHeading(0,55);
                //drive slow straight for 1/4 a second
                robot.driveForwardUseTime(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
                //place
                robot.spinCarouselMotor();
                //drive slow backwards for 1/4 a second
                robot.driveBackwardUseTime(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
                //rotate towards to -90
                robot.rotateToHeading(0,-90);
                robot.driveForwardUseFrontDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),300);
                robot.rotateToHeading(0,0);
                robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
                robot.driveBackwardUseBackDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),330);
                robot.spinCarouselMotor();
                robot.driveForwardUseBackwardDistance(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),650);
 */