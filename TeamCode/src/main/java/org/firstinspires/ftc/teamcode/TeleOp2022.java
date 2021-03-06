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
        double spinPower = 0.75;
        boolean spinToggle = true;
        boolean buttonDown = true;
        boolean spinning = false;

        boolean carouselDirection = true;
        double carouselPower = 0.75;



       //boolean spintakeMotorState = true;
       int liftMotorTicksTele = 0;
       robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //middle height is 150 ticks
        //top height is 440 ticks

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

            //GAMEPAD 1 ___________________________________________________________________________
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

            if(gamepad1.left_bumper){  //Lift motor down
                liftMotorTicksTele -= 100;
                robot.moveLiftMotor(liftMotorTicksTele,.5);
                telemetry.addData("LiftMotor Pos: ", liftMotorTicksTele);
                telemetry.update();
            }
            if(gamepad1.right_bumper){ //Lift motor up
                liftMotorTicksTele += 100;
                robot.moveLiftMotor(liftMotorTicksTele,.5);
                telemetry.addData("LiftMotor Pos: ", liftMotorTicksTele);
                telemetry.update();
            }

            if(gamepad1.y){
                robot.dropServo.setPosition(0);
            }
            else{
                robot.dropServo.setPosition(1);
            }

            if(gamepad1.b){
                while (liftMotorTicksTele < 440){
                    liftMotorTicksTele += 100;
                    robot.moveLiftMotor(liftMotorTicksTele,.5);
                    telemetry.addData("LiftMotor Pos: ", liftMotorTicksTele);
                    telemetry.update();
                }
            }

            //GAMEPAD 2 ___________________________________________________________________________
            //intake servo
            if(gamepad2.left_bumper){
                robot.intakeServo1.setPower(-1);
                robot.intakeServo2.setPower(1);
            }
            else if (gamepad2.right_bumper){
                robot.intakeServo1.setPower(1);
                robot.intakeServo2.setPower(-1);

            }
            else{
                robot.intakeServo1.setPower(0);
                robot.intakeServo2.setPower(0);
            }

            if(gamepad2.a){
                if(carouselDirection==true){
                    carouselDirection=false;
                }
                if(carouselDirection==false){
                    carouselDirection=true;
                }
            }
            if(gamepad2.y){
                ElapsedTime  carouselRuntime = new ElapsedTime();
                while(carouselRuntime.milliseconds() < 4250 && carouselDirection==true){
                    robot.carouselMotor.setPower(carouselPower);
                }
                while(carouselRuntime.milliseconds() < 4250 && carouselDirection==false){
                    robot.carouselMotor.setPower(-carouselPower);
                }
                robot.carouselMotor.setPower(0);
            }

            if(gamepad2.dpad_up && buttonDown == true) {
                buttonDown = false;
                if (spinning == false){
                    robot.spintakeMotor.setPower(spinPower);
                    spinning = true;
                }
                else{
                    robot.spintakeMotor.setPower(0);
                    spinning = false;
                }


                /*spinToggle = !spinToggle;
                if(spinToggle == true){
                    robot.spintakeMotor.setPower(spinPower);
                }
                if(spinToggle == false){
                    robot.spintakeMotor.setPower(0);
                }*/
            }

            if (!gamepad2.dpad_up){
                buttonDown = true;

            }


            if(gamepad2.dpad_down){
                //spintakeMotorState = !spintakeMotorState;
                spinPower = -spinPower;

            }







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

    //OLD CODE I DON'T WANT TO GET RID OF BUT AM NOT USING
    /*
            if(gamepad1.y){ //increment up
                liftMotorTicksTele += 100;
                robot.moveLiftMotor(liftMotorTicksTele,.5);
                telemetry.addData("LiftMotor Pos: ", liftMotorTicksTele);
                telemetry.update();
            }

            if(gamepad1.x){ //increment down
                liftMotorTicksTele -= 100;
                robot.moveLiftMotor(liftMotorTicksTele,.5);
                telemetry.addData("LiftMotor Pos: ", liftMotorTicksTele);
                telemetry.update();
            }
            */
    /* if(gamepad1.y){
                robot.dropServo.setPosition(0);
            }
            if(gamepad1.a){
                robot.dropServo.setPosition(1);
            } */
    /*
            //change carousel motor direction
            if(gamepad2.dpad_left){
                carouselDirection = -.75;
            }
            if(gamepad1.dpad_right){
                carouselDirection = .75;
            }
            //spin CarouselMotor
            if(gamepad2.y){
                ElapsedTime  carouselRuntime = new ElapsedTime();
                while(carouselRuntime.milliseconds() < 4250){
                    robot.carouselMotor.setPower(carouselDirection);
                }
                robot.carouselMotor.setPower(0);
            }
            */
    /*if(gamepad2.dpad_up){

                if(spintakeMotorState == 0){
                    spintakeMotorState = 1;
                    robot.spintakeMotor.setPower(-.75);
                }
                if(spintakeMotorState ==1){
                    spintakeMotorState = 0;
                    robot.spintakeMotor.setPower(.75);
                }
                else{
                    robot.spintakeMotor.setPower(0);
                    spintakeMotorState = 0;
                }
            } */
    /*if(spinToggle == true){
                    robot.spintakeMotor.setPower(spinPower);
                    spinToggle = false;
                }
                if(spinToggle == false){
                    robot.spintakeMotor.setPower(0);
                    spinToggle = true;
                }*/

}


