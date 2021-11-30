package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;
import java.util.Locale;
@TeleOp(name="LinearSlide Encoder Testing", group="Linear Opmode")
public class LinearSlideEncoderTesting extends LinearOpMode {


    HardwareMap2022 robot = new HardwareMap2022();

    public void runOpMode(){

        robot.init(hardwareMap);

        robot.rightLinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightLinearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        int linearSlideTicks = 0;

        waitForStart();



        while(opModeIsActive()){


            if(gamepad1.x){
                //increment LinearSlide OUT
                linearSlideTicks -= 50;
                moveLinearSlide(linearSlideTicks, .1);
            }

            if(gamepad1.b){
                //increment LinearSlide IN
                linearSlideTicks += 50;
                moveLinearSlide(linearSlideTicks, .1);
            }

            if(gamepad1.y){
                //Shoot LinearSlide ALL the way OUT
                linearSlideTicks = -800;
                moveLinearSlide(-800, .5);
            }

            if(gamepad1.a){
                //Return Linear Slide to 0
                linearSlideTicks = 0;
                moveLinearSlide(0, .5);

            }


            telemetry.addData("targetTickPos: ", linearSlideTicks);
            telemetry.addData("currentTickPos: ", robot.rightLinearSlideMotor.getCurrentPosition());
            telemetry.update();

        }


    }




    public void moveLinearSlide(int myTicks, double positivePWR){
        robot.rightLinearSlideMotor.setTargetPosition(myTicks);


        robot.rightLinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightLinearSlideMotor.setPower(positivePWR);

        while(robot.rightLinearSlideMotor.isBusy() && opModeIsActive()){
            telemetry.addData("Target: ", myTicks);
            telemetry.addData("tickPos: ",robot.rightLinearSlideMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.rightLinearSlideMotor.setPower(0);

        robot.rightLinearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
