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
/*___
<|°_°|>
 /|_|\
° |_| °
  / \
 °  ° */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MANUEL", group="Pushbot")
public class MANUEL extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Motors:
        //  left and right drive - joysticks
        //  lift - y and a
        //  delivery - b and x
        //  extension - dpad_down and dpad_up
        //  intake - R1 for on/off or R1 and R2 for on/off or just R1 to hold to be one
        //  servo????????
        //drop lift at first, drive backwards, lower lift

        double left;
        double right;
        double up;
        double down;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        up    = gamepad1.right_trigger;
        down  =gamepad1.left_trigger;


        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        if (up>=0)
            robot.bucket.setPower(up);
        else if (down>=0 && up==0)
            robot.bucket.setPower(down);
        else
            robot.bucket.setPower(0.0);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            robot.lift.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.a)
            robot.lift.setPower(robot.ARM_DOWN_POWER);
        else
            robot.lift.setPower(0.0);

        if (gamepad1.b)
            robot.delivery.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.x)
            robot.delivery.setPower(robot.ARM_DOWN_POWER);
        else
            robot.delivery.setPower(0.0);

        if (gamepad1.dpad_up)
            robot.extension.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.dpad_down)
            robot.extension.setPower(robot.ARM_DOWN_POWER);
        else
            robot.extension.setPower(0.0);

        if (gamepad1.left_bumper)
            robot.intake.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.right_bumper)
            robot.intake.setPower(robot.ARM_DOWN_POWER);
        else
            robot.intake.setPower(0.0);

        if (gamepad1.dpad_left)
            robot.storage.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.dpad_right)
            robot.storage.setPower(robot.ARM_DOWN_POWER);
        else
            robot.storage.setPower(0.0);



        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
