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


    boolean duckFound = false;
    double  duckRotateAuto = 25;
    double  duckDriveForward = 250;




    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AUOQWxb/////AAABmRP6L/V1T0Bclh/MquexUq8kKPD3h3N5sSIPraEvHInc1KyTB1KSLqkDd0mdJZibl8t7LsWmHogI6fR7p44UvkxD6uBvANg8xebRLgWIHaPvqxf3IqT8IG2VkljyPD/Unlfi357W5qXls0rtkFem3yX5kROTZEfRbmf5ZwtC3KSu6hBzriQwM7zk0zptP/MWtO6B/SZz6OWwLCR6O4I6TkKC7kQS3b1VGNonWq4fFL5jMcVPypqZKohDySdG4URcz0NqxpeEcC9P/c/VL67JKBcFaNBtix+7N/yccggWv8tUKuofNLIS1mUEv5kTzw9n4ps6ApmE2PziqmOjzpNL0MgF+V3KhRddiJjx51nFKEdX";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;



    /* Declare OpMode members. */
    HardwareMap2022 robot = new HardwareMap2022();



    private ElapsedTime     runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);







        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2, 16.0 / 9.0);

        }



        waitForStart();


        double placeHeight = getPlaceHeightTurnRight(); //moves robot to phase 2

        telemetry.addData("Place Height: ", placeHeight);
        telemetry.update();
        sleep(5000);









    }



    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.BACK;

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.minResultConfidence = 0.8f; //original
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public boolean getDuckLocation() {

        ElapsedTime duckRuntime = new ElapsedTime();

        while (duckRuntime.milliseconds() < 3000) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;

                        if ((recognition.getLabel().equalsIgnoreCase("Duck") || recognition.getLabel().equalsIgnoreCase("Cube") || recognition.getLabel().equalsIgnoreCase("Ball"))){

                            //telemetry.addData("LP", recognition.getLeft());

                            //float leftpixel;
                            //leftpixel = recognition.getLeft();
                            //telemetry.addData("LP",leftpixel);


                            duckFound = true;
                            return duckFound;
                        }
                    }
                    telemetry.update();
                }

            }

        }
        return duckFound;
    }


    public float getPlaceHeightTurnRight(){
        float funcPlaceHeight = 0;

        double startTime = runtime.milliseconds();


        robot.driveForwardUseEncoder(.3,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),duckDriveForward);

        if(getDuckLocation() == true){ // if middle confirmed
            funcPlaceHeight = 2;
        }
        else{ // else test right
            robot.rotateToHeading(0,-duckRotateAuto);
            if(getDuckLocation() == true){ //if right (highest shelf) is confirmed
                funcPlaceHeight = 3;
            }
            else{  //means it is left
                funcPlaceHeight = 1;
            }

        }
        robot.rotateToHeading(0,-90);

        telemetry.addData("Duck Detected in Place: ", funcPlaceHeight);
        telemetry.update();
        return funcPlaceHeight;
    }

    public float getPlaceHeightTurnLeft(){
        float funcPlaceHeight = 0;

        double startTime = runtime.milliseconds();


        robot.driveForwardUseEncoder(.3,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),duckDriveForward);

        if(getDuckLocation() == true){ // if middle confirmed
            funcPlaceHeight = 2;
        }
        else{ // else test right
            robot.rotateToHeading(0,duckRotateAuto);
            if(getDuckLocation() == true){ //if right (highest shelf) is confirmed
                funcPlaceHeight = 3;
            }
            else{  //means it is left
                funcPlaceHeight = 1;
            }

        }
        robot.rotateToHeading(0,90);

        telemetry.addData("Duck Detected in Place: ", funcPlaceHeight);
        telemetry.update();
        return funcPlaceHeight;
    }

}
