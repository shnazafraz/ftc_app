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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous:Red_Left", group="Pushbot")
public class AutonomousRedLeft extends LinearOpMode {
    /* Declare OpMode members. */
    org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    double buttonChoice = 0.0;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        Servo colorArm = hardwareMap.get(Servo.class, "color_arm");
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        colorArm.setPosition(1.0);
        robot.leftClaw.setPosition(-0.5);
        robot.rightClaw.setPosition(0.5);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.leftClaw.setPosition(0.5);
        robot.rightClaw.setPosition(-0.5);
        telemetry.addData("Claws", "have grabbed the block.");
        telemetry.update();


        //Working w/ color Sensor now: letting down arm
        colorSensor.enableLed(true);
        colorArm.setPosition(0.0);
        sleep(3000);
        //storing result of reading color sensor
        if(colorSensor.red() > colorSensor.blue()) {
            buttonChoice = 1.0;
            telemetry.addData("got", "red");
            telemetry.update();
        }
        else if(colorSensor.red() < colorSensor.blue()){
            buttonChoice = -1.0;
            telemetry.addData("got", "blue");
            telemetry.update();
        }
        else{
            buttonChoice = 0.0;
            telemetry.addData("[[][][]][][][]]]]", "none");
            telemetry.update();
        }
        //going back/forward to knock off ball
        robot.leftDrive.setPower(buttonChoice * FORWARD_SPEED);
        robot.rightDrive.setPower(buttonChoice * FORWARD_SPEED);
        sleep(100);
        robot.leftDrive.setPower(buttonChoice * -FORWARD_SPEED);
        robot.rightDrive.setPower(buttonChoice * -FORWARD_SPEED);
        sleep(100);
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        colorArm.setPosition(1.0);


        //now from start facing area in front of crypto box
        robot.leftDrive.setPower(FORWARD_SPEED);
        robot.rightDrive.setPower(FORWARD_SPEED);
        sleep(500);
        robot.rightDrive.setPower(0.0);
        sleep(500);
        robot.rightDrive.setPower(FORWARD_SPEED);
        sleep(400);
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        /*//get off platform
        wheelAction(0.5, FORWARD_SPEED, FORWARD_SPEED, 1);
        //turn right once
        wheelAction(0.70, TURN_SPEED, 0.0, 2);
        //forward to align with crypto-boxes
        wheelAction(0.5, FORWARD_SPEED, FORWARD_SPEED, 3);
        //turn right again, towards the crypto box
        wheelAction(0.70, TURN_SPEED, 0.0, 4);
        //go to crypto box, if aligned correctly, should be put into one of the rows
        wheelAction(1.0, FORWARD_SPEED, FORWARD_SPEED, 5);
        //drop block*/
        robot.leftClaw.setPosition(-0.5);
        robot.rightClaw.setPosition(0.5);
        telemetry.addData("Claws", "have dropped the block");
        telemetry.update();


        /*
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3
        wheelAction(3.0, FORWARD_SPEED, FORWARD_SPEED, 1);

        // Step 2:  Spin right for 1.3 seconds
        wheelAction(1.3, TURN_SPEED, -TURN_SPEED, 2);

        // Step 3:  Drive Backwards for 1 Second
        robot.leftDrive.setPower(-FORWARD_SPEED);
        robot.rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftClaw.setPosition(1.0);
        robot.rightClaw.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);*/
    }
}
