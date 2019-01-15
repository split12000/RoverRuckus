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
/*
 * Created by alissa on 4/3/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(
        name = "Mecanum Wheels 1 Controler",
        group = "Linear Opmode"
)
@Disabled
public class Mecanumwheels1Controler extends LinearOpMode {

    //Calls Robot Hardware and assigns a variable name
    Hardware2018         robot   = new Hardware2018();   // Use a Our 2018 hardware
    private ElapsedTime     runtime = new ElapsedTime();



    public void runOpMode() {

        robot.init(hardwareMap);


        waitForStart();
        runtime.reset();

        //arm pullback and release
        while (opModeIsActive()) {

            double r = Math.hypot((double) gamepad1.left_stick_x, (double) gamepad1.left_stick_y);
            double robotAngle = Math.atan2((double) gamepad1.left_stick_y, (double) gamepad1.left_stick_x) - 0.7853981633974483;
            double rightX = (double) gamepad1.right_stick_x;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            //For testing only
            /*if (gamepad1.a) {
                robot.mascot.setPosition(0);
                sleep(500);
            }
            if (gamepad1.b) {
                robot.mascot.setPosition(0.5);
                sleep(500);
            }
*/

            if (gamepad1.left_bumper) {
                robot.LiftMotor.setPower(1);
                sleep(100);
                robot.LiftMotor.setPower(0);
            }
            if (gamepad1.right_bumper) {
                robot.LiftMotor.setPower(-1);
                sleep(100);
                robot.LiftMotor.setPower(0);
            }




            if (gamepad1.b) {
                robot.latch.setPosition(.5);
            }
            if (gamepad1.x) {
                robot.latch.setPosition(0);
            }

            robot.leftFDrive.setPower(v1);
            robot.rightFDrive.setPower(v2);
            robot.leftBDrive.setPower(v3);
            robot.rightBDrive.setPower(v4);

        }
    }

            public void LSLift(double speed,
            double distance){

                int     newLSTarget;
                int     moveCounts;



                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    moveCounts = (int) (distance * Hardware2018.REVS_PER_INCH_LIFT);
                    newLSTarget = robot.LiftMotor.getCurrentPosition() + moveCounts;


                    // Set Target and Turn On RUN_TO_POSITION
                    robot.LiftMotor.setTargetPosition(newLSTarget);


                    robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    robot.LiftMotor.setPower(speed);

                    while (opModeIsActive() &&
                            robot.LiftMotor.isBusy() && robot.lSLimit1.getState() && robot.lSLimit2.getState()) {
                    }
                    robot.LiftMotor.setPower(0);
                }
    }
}

