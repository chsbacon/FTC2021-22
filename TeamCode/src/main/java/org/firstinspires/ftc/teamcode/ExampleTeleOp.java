
//these are all your imports
//they will auto generate when write something that needs and import
// don't worry too much about these, they just look scary
// you'll never actually touch them much
// normally you just collapse them visually when you're editing code anyways

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;


//The "ExampleFileOnPhone" is the file name that will show up on the phone
// The group Opmode isn't needed, but its good measure to keep it -- just keep it
@TeleOp(name = "ExampleFileOnPhone", group = "Opmode")

//Make sure @Disabled is commented out, as if it is not commented, it will not show up on the phone
//double check and make sure this is commented out, if it is uncommented, you're gonna have some trouble
//@Disabled

@Disabled
// this is where most of the code will go
//the class name, in this case, ExampleTeleOp, should match the file name just with out the .java on the end
// Extends LinearOpMode means the code will execute chronologically
public class ExampleTeleOp extends LinearOpMode {

    // references GrahamHWMap and calls it robot -- so later on you can just use robot. instead of GrahamHWMap.
    // The hardware map is where all the phone connection and device setup stuff goes
    // you'll have to replace GrahamHWMap with your hardware map file name here
    //GrahamHWMap robot = new GrahamHWMap();


    //when the init button is pressed, run the stuff in runOpMode
    @Override
    public void runOpMode() {

        //initializes the hardware map
        //robot.init(hardwareMap);

        //example variables -- you can remove these
        double x;
        double y;
        double r;

        //example variables -- you can remove these
        double launchMotorStatus = 0; // do not edit this
        double launchMotorPower = 0; //do not edit this
        double desiredLaunchPower = .75; // edit this for the power you want to motor to spin at
        double intakeMotorStatus = 0; //do not edit this
        double intakeMotorPower = 0; //do not edit this
        double desiredIntakePower = .75; //edit this for the power you want the motor to spin at



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // when the game is live -- when the robot is free to be driven -- when the driver has pressed play
        // This is where like 90% of your code will go
        while (opModeIsActive()) {
            //OpModeIsActive Start





            //example if statements -- you can remove these
            if (gamepad1.a) {

            }

            //example if statements -- you can remove these
            if (gamepad1.y) {
                sleep(250);
            }






            //example if statements -- you can remove these
            if ((gamepad2.a) && (gamepad2.left_bumper)) {
                //robot.intakeMotor.setPower(-desiredIntakePower);
            }

            //example if statements -- you can remove these
            if (gamepad2.x) {
                sleep(250);
                if (launchMotorStatus == 0) { //if motor off
                    launchMotorPower = desiredLaunchPower;  //turn motor on
                    launchMotorStatus = 1;  // motor is on
                } else if (launchMotorStatus == 1) { // if motor on
                    launchMotorPower = 0;       // turn motor off
                    launchMotorStatus = 0;      // motor is off
                }
            }




            //here is how you call a function -- you can delete this.
            ExampleFunction();





            //opModeIsActiveEnds
        }

    } //where the runOpMode ends






    // you put your functions outside of the runOpMode
    // such as below, here are some examples

    //Example Function -- you can delete this
    void ExampleFunction(){


    }

    //Example Function -- you can delete this
    void blindRotateLeft(double pwr){

        //robot.frontLeftMotor.setPower(pwr);
        //robot.frontRightMotor.setPower(pwr);
        //robot.backLeftMotor.setPower(pwr);
        //robot.backRightMotor.setPower(pwr);

    }

    //Example Function -- you can delete this
    //kills power ot all wheels
    void stopDriving(){
        //robot.frontLeftMotor.setPower(0);
        //robot.frontRightMotor.setPower(0);
        //robot.backLeftMotor.setPower(0);
        //robot.backRightMotor.setPower(0);
    }










    //these are just used for formatting angles and degrees
    // you can copy and paste this from file to file
    //It will just make your life easier, but you don't necessarily need it
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }





//where the file ends
}