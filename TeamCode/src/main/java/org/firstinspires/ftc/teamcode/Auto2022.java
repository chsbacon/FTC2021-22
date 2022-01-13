package org.firstinspires.ftc.teamcode;
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

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMap2022;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;



@Autonomous(name="Auto22", group="Opmode")
//@Disabled
public class Auto2022 extends LinearOpMode {













    /* Declare OpMode members. */
    HardwareMap2022 robot = new HardwareMap2022();




    private ElapsedTime     runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);





        int teamcolor = 0; // 1 = Blue 2 = Red
        int blue = 1;
        int red = 2;

        int side = 0; // 1 = left side start 2 = right side start
        int warehouse = 1;
        int carousel = 2;

        // Choosing the team color
        telemetry.addData("Press X for Blue, B for Red", "");
        telemetry.update();

        while (!gamepad1.x && !gamepad1.b) {
        }

        if(gamepad1.x){
            teamcolor = blue;
        }
        if(gamepad1.b){
            teamcolor = red;
        }

        telemetry.addData("teamcolor ", teamcolor);
        telemetry.update();

        // Choosing side
        telemetry.addData("Press A for warehouse, Y for carousel", "");
        telemetry.update();

        while (!gamepad1.a && !gamepad1.y) {
        }
        if (gamepad1.a) {
            side = warehouse;
        }
        if (gamepad1.y) {
            side = carousel;
        }
        telemetry.addData("side ", side);
        telemetry.update();



        waitForStart();

        if ((teamcolor == blue) && (side == warehouse)) {

            /*
            //PLACE (works but slow)
            robot.driveForwardUseEncoder(.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 125);
            robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
            robot.rotateToHeading(0,90);
            robot.driveBackwardUseEncoder(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),300);
            robot.rotateToHeading(0,160);
            robot.driveBackwardUseEncoder(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),350);
            //place on correct level
            */


            //PLACE
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),75);
            robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
            robot.rotateToHeading(0,150, 1250); //1500 safe
            robot.driveBackwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
            //place block

            //GET TO WAREHOUSE (place to warehouse)
            robot.driveForwardUseEncoder(.5, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 400);
            robot.rotateToHeading(0, 90, 1000); //1250 safe
            robot.strafeLeft(.75, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 1000);
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),750);



            //WAREHOUSE STUFF
            robot.rotateToHeading(0,85,250);
            robot.lowerIntake();
            robot.spintakeIn();
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),350);
            robot.spintakeStop();
            robot.raiseIntake();
            robot.rotateToHeading(0,90,250);
            robot.strafeLeft(1,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
            robot.driveBackwardUseAlpha(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),150,1500);



            //PLACE (2nd) (warehouse to place)
            robot.strafeRight(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),100); //to clear off wall
            robot.driveBackwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),750);
            robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),750);
            robot.rotateToHeading(0,150,1250); //1500 safe
            robot.driveBackwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
            //place block


            //GET TO WAREHOUSE (2nd)
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),400);
            robot.rotateToHeading(0, 90,1000);  //1250 safe
            robot.strafeLeft(.75, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 1000); //1250 safe
            robot.driveForwardUseEncoder(.75, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 1000);





        }

        if((teamcolor == blue) && (side == carousel)){
            //PLACE
            robot.driveForwardUseEncoder(.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 125);
            robot.rotateToHeading(0, 40,1500);
            robot.driveForwardUseEncoder(.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 700);
            //place on correct level

            robot.driveBackwardUseEncoder(.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 325);
            robot.rotateToHeading(0, -90,1500);
            robot.driveForwardUseEncoder(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),1000);
            robot.rotateToHeading(0,0,2000);
            robot.strafeRight(.6,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),1000);

            robot.driveForwardUseBlue(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),42,1000);
            robot.driveBackwardUseEncoder(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),350);
            //spin carousel motor
            robot.driveForwardUseBlue(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),42,2000);
        }












        if((teamcolor == red) && (side == warehouse)){

        }


        if((teamcolor == red) && (side == carousel)){
        }





    }


}
