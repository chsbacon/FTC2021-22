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
import com.qualcomm.robotcore.hardware.CRServo;
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
        
        double carouselServoOnPower = -1;

        int linearSlideTicks = 0;

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

            /*
            FUNCTION EXAMPLES

            // to drive straight for a set time at a set speed
            robot.driveStraightTime(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),5000);
                    //to grab current heading from robot (straight
                    //robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)

            // to rotate to a certain heading
                (set pwr to 0)
            robot.rotateToHeading(0,-135);


             */



            //grabs current orientation for this iteration of opModeIsActive
            currentOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //GAMEPAD 1 Capabilities
            
            //fast slow toggle
            if(gamepad1.a){
                fastSlow = 2;
            }
            else{
                fastSlow = 1;
            }

            if(gamepad1.dpad_left){
                //rotate 90 deg left
                robot.rotateToHeading(0.25,90);
            }
            if(gamepad1.dpad_right){
                //rotate 90 deg right
                robot.rotateToHeading(0.25,-90);
            }
            
            //GAMEPAD 2 Capabilities

            if(gamepad2.y){
                robot.spinCarouselMotor();
            }
            if(gamepad2.a){
                //dump cargo
            }
            if(gamepad2.dpad_up){
                //linear slide out
            }
            if(gamepad2.dpad_down){
                //linear slide in
            }
            if(gamepad2.b){
                //linear slide up down toggle
            }
            if(gamepad2.x){
                robot.spintake();
            }


            
            //driving controls
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


}
