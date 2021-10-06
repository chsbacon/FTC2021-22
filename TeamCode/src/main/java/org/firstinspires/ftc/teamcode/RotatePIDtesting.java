package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;
import java.util.Locale;
@TeleOp(name="RotatePIDtesting", group="Linear Opmode")
public class RotatePIDtesting extends LinearOpMode {


    HardwareMap2022 robot = new HardwareMap2022();

    public void runOpMode(){

        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            double kP = .475;
            double kI = .34;
            double kD = .52;

            if(gamepad1.x){
                sleep(250);
                kP = kP + .01;
            }

            if((gamepad1.x) && (gamepad1.left_bumper)){
                sleep(250);
                kP = kP - .01;
            }


            if(gamepad1.y){
                sleep(250);
                kI = kI + .01;
            }
            if((gamepad1.y) && (gamepad1.left_bumper)){
                sleep(250);
                kI = kI - .01;
            }

            if(gamepad1.a){
                rotateToHeadingV1(.5, 90, kP, kI, kD);
            }


            if(gamepad1.b){
                sleep(250);
                kD = kD + .01;
            }
            if((gamepad1.b) && (gamepad1.left_bumper)){
                sleep(250);
                kD = kD - .01;
            }

            if(gamepad1.a){
                rotateToHeadingV1(.25, 90, kP, kI, kD);
            }

            telemetry.addData("kP: ", "%.2f", kP);
            telemetry.addData("kI: ", "%.2f", kI);
            telemetry.addData("kD: ", "%.2f", kD);

            telemetry.update();

        }


    }


    void stopDriving(){
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }



    void rotateToHeadingV1(double pwr, double target, double kP, double kI, double kD){

        // set to a big number so it doesn't accidentally match the target angle
        //therefore hypothetically completing the while-loop accidentally
        double currAng = 10000;


        Orientation currOrient;

        double integralSum = 0;
        double lastError = 0;
        double error;
        double derivative;
        double out;

        ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



        while((currAng != target) && opModeIsActive()){

            pidTimer.reset();

            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            error = target - currAng;
            derivative = (error - lastError) / pidTimer.milliseconds();
            integralSum = integralSum + (error * pidTimer.time());
            out = (kP * error) + (kI * integralSum) + (kD * derivative);


            telemetry.addData("target: ", "%.2f", target);
            telemetry.addData("current: ", "%.2f", currAng);
            telemetry.addData("out: ", "%.2f", out);
            telemetry.update();

            //robot.frontLeftMotor.setPower(pwr);
            //robot.frontRightMotor.setPower(pwr);
            //robot.backLeftMotor.setPower(pwr);
            //robot.backRightMotor.setPower(pwr);

            robot.frontLeftMotor.setPower(pwr + out);
            robot.frontRightMotor.setPower(pwr + out);
            robot.backLeftMotor.setPower(pwr + out);
            robot.backRightMotor.setPower(pwr + out);

            lastError = error;

        }
        stopDriving();
        telemetry.addData("target: ", "%.2f", target);
        telemetry.addData("current: ", "%.2f", currAng);
        telemetry.update();
    }



}
