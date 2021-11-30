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
@TeleOp(name="Lift Motor Encoder Testing", group="Linear Opmode")
public class LiftMotorEncoderTesting extends LinearOpMode {


    HardwareMap2022 robot = new HardwareMap2022();

    public void runOpMode(){

        int liftMotorTicks = 0;
        //robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        waitForStart();



        while(opModeIsActive()){


           if(gamepad1.y){
               liftMotorTicks += 5;
               moveLiftMotor(liftMotorTicks, .2);
           }

           if(gamepad1.a){
               liftMotorTicks -= 5;
               moveLiftMotor(liftMotorTicks,.2);
           }



        }


    }




    public void moveLiftMotor(int myTicks, double positivePWR){
        //robot.liftMotor.setTargetPosition(myTicks);

        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.liftMotor.setPower(positivePWR);

        while(/*robot.liftMotor.isBusy() && */opModeIsActive()){
            telemetry.addData("Target: ", myTicks);
            //telemetry.addData("tickPos: ",robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        //robot.liftMotor.setPower(0);

        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
