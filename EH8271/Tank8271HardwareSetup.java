package org.firstinspires.ftc.teamcode.ExampleCode.EH8271;

/* Copyright (c) 2019 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.teamcode.AutoOptions.AutonomousBooleanOption;
import org.firstinspires.ftc.teamcode.AutoOptions.AutonomousIntOption;
import org.firstinspires.ftc.teamcode.AutoOptions.AutonomousOption;
import org.firstinspires.ftc.teamcode.AutoOptions.AutonomousTextOption;



/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutonomousTSE", group = "TFOD-Autos")
//@Disabled
public class AutonomousTankTSE extends LinearOpMode {

    Tank8271HardwareSetup robot = new Tank8271HardwareSetup();

    //region Initialize TFOD and VuForia
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "TSE_Complete.tflite";
    private static final String[] LABELS = {
            "TSE"
    };

    String barcode = null;

    int bottomLevel = 350;
    int approachBottom = -390;

    int middleLevel = 600;
    int approachMiddle = -400;

    int topLevel = 1100;
    int approachTop = -600;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Ad/WJUD/////AAABmaHL2VlsskF7gs94CBhc85VMCMsnduT4r56lJ6R1ADz06l0nCXlkYHuEr/9MViHanSiKcefbD5RMEKuNSbMTOmC8JGbEkiQB5a+kE/JDCayLu/0cAj7+y4wkNo2v4YtJlr1YJ5HCLZ1Rzv007cx4S+NbSv3TSxZUQzomnBbZIc/3uLx5S0Sr3eood8gq7xRVTwXh0Rp9GJk+my8sz87vJyg+nZlWXa3q5WzuS0YRq2F5XMDMH1opYjN3Ub+0xFIZO82tBSBQfAMGLruFRyjQ7qpVgPra19wu8PldMmHoGHPdQgT+G6iAGCjClGpcnPtZMXw1VycsGRyjH4pBSH12J5HIheL9b/BTvvBwelC+0FeC";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    //endregion

    //region analyze barcode
    public String analyzeBarcode()
    {
        if(tfod != null)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if(updatedRecognitions != null)
            {
                for (Recognition recognition : updatedRecognitions)
                {
                    if(recognition.getConfidence() > .89)
                    {
                        if(recognition.getLabel().equals("TSE"))
                        {
                            //this will only identify the two right barcodes out of all three
                            //if Team Element is on the left-half of the camera
                            if(recognition.getRight() < 400)
                            {
                                return "B";
                            }

                            //if Team Element is on the right-half of the camera
                            else if(recognition.getRight() > 400)
                            {
                                return "C";
                            }
                        }
                    }
                }
            }
        }
        //else if Team Element is neither on left or right side, assume it's the far left Team Element
        return "A";
    }
    //endregion

    // For each auto option the parameters are essentially 1- the label to show on the driver station, 2 - starting value, 3 - the possible values
    AutonomousTextOption    allianceColor       = new AutonomousTextOption("Alliance Color", "blue", new String[] {"blue", "red"});
    AutonomousTextOption    startPos       = new AutonomousTextOption("Start Position", "X", new String[] {"X", "Y"});
    AutonomousTextOption    endPos = new AutonomousTextOption("End Position", "storage", new String[] {"storage","warehouse"});
    AutonomousIntOption     waitStart           = new AutonomousIntOption("Wait at Start", 0, 0, 20);

    //This is the order of our options and setting them all to their preset value.
    AutonomousOption[] autoOptions       = {allianceColor, startPos, endPos, waitStart};
    int currentOption = 0;

    //this setting the buttons to false to make sure options are not being chosen for us.
    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;
    boolean yPressed = false;

    //region Autonomous Options
    // This is how we get our autonomous options to show up on our phones.
    public void showOptions (){
        int index = 0;
        String str = "";

        while (index < autoOptions.length){
            switch (autoOptions[index].optionType){
                case STRING:
                    str = ((AutonomousTextOption)autoOptions[index]).getValue();
                    break;
                case INT:
                    str = Integer.toString(((AutonomousIntOption)autoOptions[index]).getValue());
                    break;
                case BOOLEAN:
                    str = String.valueOf(((AutonomousBooleanOption)autoOptions[index]).getValue());
                    break;
            }

            if (index == currentOption){
                telemetry.addData(Integer.toString(index) + ") ==> " + autoOptions[index].name,str);
            } else {
                telemetry.addData(Integer.toString(index) + ")     " + autoOptions[index].name, str);
            }

            index = index + 1;
        }
        telemetry.update();
    }
    // This is how we select our auto options
    public void selectOptions () {

        while (currentOption< autoOptions.length && !opModeIsActive()){
            showOptions();

            if (gamepad1.a && !aPressed) {
                currentOption = currentOption + 1;
            }
            aPressed = gamepad1.a;

            if (gamepad1.y && !yPressed) {
                currentOption = currentOption - 1;
            }
            yPressed = gamepad1.y;

            if (gamepad1.b && !bPressed) {
                autoOptions[currentOption].nextValue();
            }
            bPressed = gamepad1.b;

            if (gamepad1.x && !xPressed) {
                autoOptions[currentOption].previousValue();
            }
            xPressed = gamepad1.x;

            telemetry.update();
            Thread.yield();
        }

        telemetry.addData("Robot","READY!!");
        telemetry.update();
    }
    //endregion

    void BlueDuck() throws InterruptedException
    {
        while(robot.rightPanel.isPressed())
        {
            robot.motorFR.setPower(0.3);
            robot.motorBR.setPower(-0.3);
            robot.motorFL.setPower(0.3);
            robot.motorBL.setPower(-0.3);
        }
        StopDrivingTime(100);

        BlueDuckSpin(2500);
        StopDrivingTime(1000);
        BlueDuckStop();
    }

    void RedDuck() throws InterruptedException
    {
        while(robot.leftPanel.isPressed())
        {
            robot.motorFR.setPower(-0.3);
            robot.motorBR.setPower(0.3);
            robot.motorFL.setPower(-0.3);
            robot.motorBL.setPower(0.3);
        }
        StopDrivingTime(100);

        RedDuckSpin(2500);
        StopDrivingTime(1000);
        RedDuckStop();
    }

    private void Blue1() throws InterruptedException
    {
        DriveBackwardEncoder(0.3, 800);
        StopDrivingTime(500);

        liftArm(DRIVE_POWER, 1000);
        maintainArmPos();

        DriveForwardEncoder(0.2, -300);
        StopDrivingTime(500);

        SpinRightEncoder(0.3, 750);
        maintainArmPos();
        StopDrivingTime(500);

        DriveForwardEncoder(0.4, -2500);
        maintainArmPos();
        StopDrivingTime(100);

        BlueDuck();

        DriveForwardEncoder(DRIVE_POWER, -500);
        extendArm(-DRIVE_POWER, 500);
        maintainArmPos();

        if(endPos.getValue().equals("warehouse"))
        {
            StrafeLeftEncoder(0.3, 500);
            DriveBackwardEncoder(0.9, 6000);
        }
        else if(endPos.getValue().equals("storage"))
        {
            DriveBackwardEncoder(DRIVE_POWER, 150);
            maintainArmPos();
            StopDrivingTime(100);

            StrafeLeftEncoder(0.3, 800);

            extendArm(-DRIVE_POWER, 500);
            maintainArmPos();

            DriveForwardEncoder(0.3, -500);
            maintainArmPos();
        }

    }

    private void Blue2() throws InterruptedException
    {
        DriveBackwardEncoder(0.4, 300);
        StopDrivingTime(100);

        SpinLeftEncoder(0.3, -820);
        maintainArmPos();
        StopDrivingTime(500);

        StrafeLeftEncoder(0.3, 1000);

        DriveForwardEncoder(0.4, -2500);
        maintainArmPos();

        StopDriving();
    }

    private void Red1() throws InterruptedException
    {
        DriveBackwardEncoder(0.3, 800);
        StopDrivingTime(500);

        liftArm(DRIVE_POWER, 1000);
        maintainArmPos();

        DriveForwardEncoder(0.2, -300);
        StopDrivingTime(500);

        SpinLeftEncoder(0.3, -750);
        maintainArmPos();
        StopDrivingTime(500);

        DriveForwardEncoder(0.4, -2500);
        maintainArmPos();

        RedDuck();

        DriveForwardEncoder(DRIVE_POWER, -500);
        extendArm(-DRIVE_POWER, 500);
        maintainArmPos();

        if(endPos.getValue().equals("warehouse"))
        {
            StrafeRightEncoder(0.3, -500);
            DriveBackwardEncoder(0.9, 6000);
        }
        else if(endPos.getValue().equals("storage"))
        {
            DriveBackwardEncoder(DRIVE_POWER, 150);
            maintainArmPos();
            StopDrivingTime(100);

            StrafeRightEncoder(0.3, -800);

            extendArm(-DRIVE_POWER, 500);
            maintainArmPos();

            DriveForwardEncoder(0.3, -500);
            maintainArmPos();
        }
        StopDriving();
    }

    private void Red2() throws InterruptedException
    {
        DriveBackwardEncoder(0.4, 300);
        StopDrivingTime(100);

        SpinRightEncoder(0.3, 820);
        maintainArmPos();
        StopDrivingTime(500);

        StrafeRightEncoder(0.3, -1000);

        DriveForwardEncoder(0.4, -2500);
        maintainArmPos();

        StopDriving();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        robot.init(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }

        selectOptions();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        sleep(waitStart.getValue()*1000);

        barcode = analyzeBarcode();

        if(startPos.getValue().equals("Y"))
        {
            if(allianceColor.getValue().equals("blue"))
            {
                //If TSE is on middle barcode
                if (barcode.equals("B")) {
                    liftArm(DRIVE_POWER, middleLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachMiddle);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    liftArm(DRIVE_POWER, 500);

                    Blue1();

                    StopDriving();
                }

                //If TSE is on right barcode
                else if (barcode.equals("C")) {
                    liftArm(DRIVE_POWER, topLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachTop);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    maintainArmPos();

                    Blue1();

                    StopDriving();
                }

                //If TSE is on left barcode
                else {
                    liftArm(DRIVE_POWER, bottomLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachBottom);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    liftArm(DRIVE_POWER, 900);

                    Blue1();

                    StopDriving();
                }
            }
            else if(allianceColor.getValue().equals("red"))
            {
                //If TSE is on middle barcode
                if (barcode.equals("B")) {
                    liftArm(DRIVE_POWER, middleLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachMiddle);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    liftArm(DRIVE_POWER, 500);

                    Red1();

                    StopDriving();
                }

                //If TSE is on right barcode
                else if (barcode.equals("C")) {
                    liftArm(DRIVE_POWER, topLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachTop);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Red1();

                    StopDriving();
                }

                //If TSE is on left barcode
                else {
                    liftArm(DRIVE_POWER, bottomLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachBottom);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    liftArm(DRIVE_POWER, 900);

                    Red1();

                    StopDriving();
                }
            }
        }

        if(startPos.getValue().equals("X"))
        {
            if(allianceColor.getValue().equals("blue"))
            {
                //If TSE is on middle barcode
                if (barcode.equals("B")) {
                    liftArm(DRIVE_POWER, middleLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachMiddle);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Blue2();

                    StopDriving();
                }

                //If TSE is on right barcode
                else if (barcode.equals("C")) {
                    liftArm(DRIVE_POWER, topLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachTop);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Blue2();

                    StopDriving();
                }

                //If TSE is on left barcode
                else {
                    liftArm(DRIVE_POWER, bottomLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachBottom);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Blue2();

                    StopDriving();
                }
            }

            else if(allianceColor.getValue().equals("red"))
            {
                //If TSE is on middle barcode
                if (barcode.equals("B")) {
                    liftArm(DRIVE_POWER, middleLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachMiddle);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Red2();

                    StopDriving();
                }

                //If TSE is on right barcode
                else if (barcode.equals("C")) {
                    liftArm(DRIVE_POWER, topLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachTop);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Red2();

                    StopDriving();
                }

                //If TSE is on left barcode
                else {
                    liftArm(DRIVE_POWER, bottomLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachBottom);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Red2();

                    StopDriving();
                }
            }
        }

        //region Telemetry for TFOD
/*
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    barcode = analyzeBarcode();

                    //I want to add this to display what barcode position it's on, however I'm unsure where to put it
                    telemetry.addData("Position", barcode);
                    telemetry.update();

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


                        }
                    }
                }
            }
        }

 */
        //endregion
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //region Drive Functions
    double DRIVE_POWER = 0.5;

    //region Time Driving Functions
    public void DriveForward(double power)
    {
        // write the values to the motors
        robot.motorFR.setPower(-power);//still need to test motor directions for desired movement
        robot.motorFL.setPower(power);
        robot.motorBR.setPower(-power);
        robot.motorBL.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }

    public void StopDrivingTime(long time) throws InterruptedException
    {
        robot.motorFR.setPower(0);//still need to test motor directions for desired movement
        robot.motorFL.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorBL.setPower(0);
        Thread.sleep(time);
    }

    public void StrafeLeft(double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFR.setPower(-power);
        robot.motorFL.setPower(-power);
        robot.motorBR.setPower(power);
        robot.motorBL.setPower(power);
        Thread.sleep(time);
    }

    public void StrafeRight(double power, long time) throws InterruptedException
    {
        StrafeLeft(-power, time);
    }

    public void SpinRight (double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFR.setPower(power);
        robot.motorFL.setPower(power);
        robot.motorBR.setPower(power);
        robot.motorBL.setPower(power);
        Thread.sleep(time);
    }

    public void SpinLeft (double power, long time) throws InterruptedException
    {
        SpinRight(-power, time);
    }

    //region DuckSpins
    public void BlueDuckSpin(long time) throws InterruptedException
    {
        robot.duckBlue.setPower(0.7);
        Thread.sleep(time);
    }

    public void BlueDuckStop() throws InterruptedException
    {
        robot.duckBlue.setPower(0);
    }

    public void RedDuckSpin(long time) throws InterruptedException
    {
        robot.duckRed.setPower(-0.7);
        Thread.sleep(time);
    }

    public void RedDuckStop() throws InterruptedException
    {
        robot.duckRed.setPower(0.5);
    }
    //endregion

    public void StopDriving()
    {
        DriveForward(0);
    }

    //endregion

    //region Encoder Drive Functions
    public void StrafeRightEncoder(double power, int pos)
    {
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFR.setTargetPosition(pos);
        robot.motorBR.setTargetPosition(pos);
        //robot.motorFL.setTargetPosition(pos);
        robot.motorBL.setTargetPosition(pos);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFR.setPower(power);
        robot.motorBR.setPower(-power);
        //robot.motorFL.setPower(power);
        robot.motorBL.setPower(-power);

        while(robot.motorBR.getCurrentPosition() > pos)
        {
            robot.motorFL.setPower(power);

            telemetry.addData("FRmotorPos", robot.motorFR.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFL.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBR.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFR.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBL.setPower(0);

    }

    public void StrafeLeftEncoder(double power, int pos)
    {
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFR.setTargetPosition(pos);
        robot.motorBR.setTargetPosition(pos);
        //robot.motorFL.setTargetPosition(pos);
        robot.motorBL.setTargetPosition(pos);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFR.setPower(-power);
        robot.motorBR.setPower(power);
        //robot.motorFL.setPower(power);
        robot.motorBL.setPower(power);

        while(robot.motorBR.getCurrentPosition() < pos)
        {
            robot.motorFL.setPower(-power);

            telemetry.addData("FRmotorPos", robot.motorFR.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFL.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBR.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFR.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBL.setPower(0);

    }

    public void DriveForwardEncoder(double power, int pos)
    {
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFR.setTargetPosition(pos);
        robot.motorBR.setTargetPosition(pos);
        //robot.motorFL.setTargetPosition(pos);
        robot.motorBL.setTargetPosition(pos);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFR.setPower(-power);
        robot.motorBR.setPower(-power);
        //robot.motorFL.setPower(power);
        robot.motorBL.setPower(power);

        while(robot.motorBR.getCurrentPosition() > pos)
        {
            robot.motorFL.setPower(power);

            telemetry.addData("FRmotorPos", robot.motorFR.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFL.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBR.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFR.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBL.setPower(0);

    }

    public void DriveBackwardEncoder(double power, int pos)
    {
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFR.setTargetPosition(pos);
        robot.motorBR.setTargetPosition(pos);
        //robot.motorFL.setTargetPosition(pos);
        robot.motorBL.setTargetPosition(pos);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFR.setPower(power);
        robot.motorBR.setPower(power);
        //robot.motorFL.setPower(power);
        robot.motorBL.setPower(-power);

        while(robot.motorBR.getCurrentPosition() < pos)
        {
            robot.motorFL.setPower(-power);

            telemetry.addData("FRmotorPos", robot.motorFR.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFL.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBR.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFR.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBL.setPower(0);

    }

    public void SpinLeftEncoder(double power, int pos)
    {
        //-750 is a 90 degree left turn

        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFR.setTargetPosition(pos);
        robot.motorBR.setTargetPosition(pos);
        //robot.motorFL.setTargetPosition(pos);
        robot.motorBL.setTargetPosition(pos);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFR.setPower(-power);
        robot.motorBR.setPower(-power);
        //robot.motorFL.setPower(power);
        robot.motorBL.setPower(-power);

        while(robot.motorBR.getCurrentPosition() > pos)
        {
            robot.motorFL.setPower(-power);

            telemetry.addData("FRmotorPos", robot.motorFR.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFL.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBR.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFR.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBL.setPower(0);

    }

    public void SpinRightEncoder(double power, int pos)
    {
        //750 is a 90 degree right turn

        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFR.setTargetPosition(pos);
        robot.motorBR.setTargetPosition(pos);
        //robot.motorFL.setTargetPosition(pos);
        robot.motorBL.setTargetPosition(pos);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFR.setPower(power);
        robot.motorBR.setPower(power);
        //robot.motorFL.setPower(power);
        robot.motorBL.setPower(power);

        while(robot.motorBR.getCurrentPosition() < pos)
        {
            robot.motorFL.setPower(power);

            telemetry.addData("FRmotorPos", robot.motorFR.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFL.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBR.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFR.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBL.setPower(0);

    }
    //endregion

    //region Arm & Hand Functions
    public void openHands() throws InterruptedException
    {
        robot.servoHandL.setPosition(0.4);
        robot.servoHandR.setPosition(0.4);
    }

    public void liftArm(double power, int pos) throws InterruptedException
    {
        //sets arm's starting pos to 0
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armMotor.setTargetPosition(pos);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(power);
        while(robot.armMotor.isBusy())
        {
            telemetry.addData("armPos", robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void maintainArmPos() throws InterruptedException
    {
        liftArm(DRIVE_POWER, 0);
    }

    public void extendArm(double power, long time) throws InterruptedException
    {
        robot.slideMotor.setPower(power);
        Thread.sleep(time);
    }
    //endregion
    //endregion
}