// FTC Team 7080 BACON
// Autonomous code 2021-2022

package org.firstinspires.ftc.teamcode;


//imports related to the opmode

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.HardwareBACONbot;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;

import java.util.Locale;

//_________________________________________________________________________________________________


@Autonomous(name = "BACON: Autonomous 21-22", group = "Opmode")
//@Disabled


public class BACONmechAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    HardwareMap2022 robot = new HardwareMap2022();

    //variables

    public void runOpMode() {

        int teamcolor = 0; // 1 = Blue 2 = Red
        int blue = 1;
        int red = 2;


        int side = 0; // 1 = left side start 2 = right side start
        int warehouse = 1;
        int carousel = 2;

        double meetDistance = 860; //Distance from wall to the rings (CM From Wall (BackSensor))

        double lastTime = runtime.milliseconds();

        // wobbleServo and wobbleMotor states
        float grabPos = 0;
        float freePos = 1;
        float upTilt = 0;
        float downTilt = 1;

        Orientation angles;
        Acceleration gravity;

        robot.init(hardwareMap);

        // Choosing the team color
        telemetry.addData("Press X for Blue, B for Red", "");
        telemetry.update();
        //Call component setup functions here: ex. openClaw, raiseLauncher, etc.

        //It will only assign color if the buttons are pressed
        while (!gamepad1.x && !gamepad1.b) {
        }



        telemetry.addData("teamcolor ", teamcolor);
        telemetry.update();

        // Choosing task
        telemetry.addData("Press A for drop&park, Y for fullRun", "");
        telemetry.update();
        while (!gamepad1.a && !gamepad1.y) {
        }
        if (gamepad1.a) {
            task = dropPark;
        }
        if (gamepad1.y) {
            task = fullRun ;
        }
        telemetry.addData("task ", task);
        telemetry.update();

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) robot.backDistance;

        //Wobble grabber position
        robot.wobbleServo.setPosition(grabPos);
        //robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();


        double WSrestPos1 = 0;

        // Don't burn CPU cycles busy-looping in this sample
        //sleep(50);

        // run until the end of the match (when driver presses STOP)

        if (teamcolor==red && side==warehouse){

        }
        if (teamcolor==red && side==carousel){

        }
        if (teamcolor==blue && side==warehouse){

        }
        if (teamcolor==blue && side==carousel){
            //sense
            robot.strafeRightUsingRightDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES),500);
            robot.driveForwardUseBackwardDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES),200);
            robot.spinCarouselMotor();
            robot.strafeLeftUsingRightDistance(0.25,);

        }




        public void verticalSlide (duckPlace){
        if duckPlace == {
                linearUp.setPower(-1);
            while ((linearUp.getCurrentPosition() > -1500) && opModeIsActive()) {
                telemetry.addData("verticalSlide pos: ", linearUp.getCurrentPosition());
                telemetry.update();
        }
        linearUp.setPower(0.0);
        }
        if duckPlace == {

        }
        if duckPlace == {

        }
    }

