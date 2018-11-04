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

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.VuMarkTargetResult;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Blue_Relic", group ="Blue")
//@Disabled
public class Blue_Relic extends LinearOpMode {

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    public static final String TAG = "Vuforia VuMark Sample";

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     SLOW          = 0.2;
    double                  clawPosition    = 270;

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor arm = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo bottomLeft = null;
    public Servo bottomRight = null;
    //private Servo relic = null;
    //private Servo hook = null;
    public Servo jewel = null;
    public NormalizedColorSensor colorSensor;

    OpenGLMatrix lastLocation = null;

    View relativeLayout;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() throws InterruptedException{

        boolean lastResetState = false;
        boolean curResetState  = false;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        bottomLeft = hardwareMap.get(Servo.class, "bottomLeft");
        bottomRight = hardwareMap.get(Servo.class, "bottomRight");
        ColorSensor colorSensor    =  hardwareMap.get(ColorSensor.class, "sensor_color");
        jewel     = hardwareMap.get(Servo.class, "color");
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        leftClaw.setPosition(.5);
        rightClaw.setPosition(.5);
        jewel.setPosition(1);

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        // bPrevState and bCurrState keep track of the previous and current state of the button
        boolean bPrevState = false;
        boolean bCurrState = false;

        // Get a reference to our sensor object.


        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            // OR...  Do Not Activate the Camera Monitor View, to save power
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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
         * and paste it in to your code onthe next line, between the double quotes.
         */
            parameters.vuforiaLicenseKey = "Adt026z/////AAAAmcFEiUnuGURbkBcTUYKGvx0sTeUov6Bg5+Ms6+A2939igu+3N/ReTEIyz/lC9TKV1zvxYMqS2kbnDHie3svei7IAAC6gRUHXmkPkZZ50qiqJQO6m5R9Esz3GQRp1SWIuiXT8HGT3O940bzQdycONo/O8eHatcjAnQfEYLGzp1TIOnHm3a/VmMl9Sy6a64t2k8Bf0Vx8K9okP7uBVRm61PQRXQp5KYBCWkdqPNe6xmitJNF6aqMBOLDD1B6Nsqgi0eBMY9gQPFvAy22FT28sYh/i/qp3RB3LVwN/TRCdJDb0349NDy7C0HKx3AGdehMpc/V8MHwErvXqMX+qvtyXlRWcgOWzZUUK0tn9Om5QGVvzw";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            /**
             * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
             * in this data set: all three of the VuMarks in the game were created from this one template,
             * but differ in their instance id information.
             * @see VuMarkInstanceId
             */
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            runtime.reset();
            while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
                telemetry.addData("calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
                telemetry.update();
                sleep(50);
            }

            telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
            telemetry.clear(); telemetry.update();

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            waitForStart();

            relicTrackables.activate();

            while (opModeIsActive()) {

                if (bCurrState != bPrevState) {
                    // If the button is (now) down, then toggle the light
                    if (bCurrState) {
                        if (colorSensor instanceof SwitchableLight) {
                            SwitchableLight light = (SwitchableLight)colorSensor;
                            light.enableLight(!light.isLightOn());
                        }
                    }
                }
                bPrevState = bCurrState;

               // NormalizedRGBA colors = colorSensor.getNormalizedColors();

                /**
                 * See if any of the instances of {@link relicTemplate} are currently visible.
                 * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                 * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                 * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                 */
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                    telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                    double boxTime;
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;

                    }

                }

                else {
                    telemetry.addData("VuMark", "not visible");
                }

                telemetry.update();

                //Insert Program Directions Here

                clawPosition = Range.clip(clawPosition, 0, 270);

                clawPosition = 0;

                //Insert Program Directions Here

                //Step 1: Move Jewel down
                jewel.setPosition( clawPosition);
                leftClaw.setPosition(1);
                rightClaw.setPosition(0);
                bottomLeft.setPosition(.5);
                bottomRight.setPosition(.5);
                Thread.sleep (5000 );
                arm.setPower(-.75);
                Thread.sleep(350);
                arm.setPower(0);

                if (colorSensor.red() > colorSensor.blue()) {
                    Drive(.2, 1000);
                    jewel.setPosition(270);
                    Thread.sleep(1000);
                    TurnLeft(.4, 500);
                    Drive(.4, 1000);
                }
                else if (colorSensor.blue()> colorSensor.red()) {
                    Reverse(.2, 1000);
                    jewel.setPosition(270);
                    Thread.sleep(1000);
                    Drive (.4, 1750);
                }
                else if (colorSensor.blue() == colorSensor.red()){
                    Drive(.1, 500);
                     if (colorSensor.red() > colorSensor.blue()) {
                         Drive(.2, 750);
                         jewel.setPosition(270);
                         Thread.sleep(1000);
                         Drive(.4, 1000);
                     }
                     else if (colorSensor.blue()> colorSensor.red()) {
                          Reverse(.2, 750);
                          jewel.setPosition(270);
                          Thread.sleep(1000);
                          Drive (.4, 1500);
                }}

                idle();
                stop();

                //Step 2: Identify Color of Jewel and Vuforia path
               /* if (colorSensor.red() > colorSensor.blue()){
                    if (vuMark == RelicRecoveryVuMark.RIGHT){
                        Drive (.2, 350);
                        Reverse(.4, 150);
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        Reverse (.3, 1250);

                    }

                    else if (vuMark == RelicRecoveryVuMark.CENTER){
                        Drive (.2, 350);
                        Reverse(.4, 150);
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        Reverse (.3, 1500);

                    }
                    else if (vuMark == RelicRecoveryVuMark.LEFT){
                        Drive (.2, 350);
                        Reverse(.4, 150);
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        Reverse(.3, 1750);

                    }
                    else if (vuMark == RelicRecoveryVuMark.UNKNOWN){
                        Drive (.2, 350);
                        Reverse(.4, 150);
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        Reverse(.3,1500);

                    }

                }

                else if (colorSensor.blue() > colorSensor.red()){
                    if (vuMark == RelicRecoveryVuMark.RIGHT){
                        Reverse(.2, 350);
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        Reverse (.3, 1250);

                    }
                    else if (vuMark == RelicRecoveryVuMark.CENTER){
                        Reverse(.2, 350);
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        Reverse (.3, 1500);

                    }
                    else if (vuMark == RelicRecoveryVuMark.LEFT){
                        Reverse(.2, 350);
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        Reverse (.3, 1750);

                    }
                    else if (vuMark == RelicRecoveryVuMark.UNKNOWN){
                        Reverse(.2, 350);
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        Reverse (.3, 1500);

                    }

                }*/
               // else if (colorSensor.blue() == colorSensor.red()){
                    /*if (vuMark == RelicRecoveryVuMark.RIGHT){
                        Reverse(2.5);
                        idle();
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        idle();
                        TurnLeft(1);
                        idle();

                    }
                    else if (vuMark == RelicRecoveryVuMark.CENTER){
                        Reverse(2.5);
                        idle();
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        idle();
                        TurnLeft(1);
                        idle();

                    }
                    else if (vuMark == RelicRecoveryVuMark.LEFT){
                        Reverse(2.5);
                        idle();
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        idle();
                        TurnLeft(1);
                        idle();

                    }
                    else if (vuMark == RelicRecoveryVuMark.UNKNOWN){
                        Reverse(2.5);
                        idle();
                        clawPosition = 270;
                        jewel.setPosition(clawPosition);
                        idle();
                        TurnLeft(1);
                        idle();

                    }*/
                }
                idle();
                stop();

            }

        }


    public void Drive (double Power, long Time) throws InterruptedException{

        leftDrive.setPower(-Power);
        rightDrive.setPower(Power);
        Thread.sleep(Time);

    }

    public void Reverse (double Power, long Time) throws InterruptedException{

        leftDrive.setPower(Power);
        rightDrive.setPower(-Power);
        Thread.sleep(Time);
    }

    public void TurnRight (double Power, long Time) throws InterruptedException{

        leftDrive.setPower(Power);
        rightDrive.setPower(Power);
        Thread.sleep(Time);

    }

    public void TurnLeft (double Power, long Time)throws InterruptedException{

        leftDrive.setPower(-Power);
        rightDrive.setPower(-Power);
        Thread.sleep(Time);

    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
