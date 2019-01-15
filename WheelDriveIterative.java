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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(
        name = " Mecanum Iterative (USE ME!!!)",
        group = "Opmode"
)
//@Disabled
public class WheelDriveIterative extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Calls Robot Hardware and assigns a variable name
    Hardware2018         robot   = new Hardware2018();   // Use a Our 2018 hardware
    double armliftlimit;


    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void start() {
        runtime.reset();
        //set limit up for arm.
        robot.ArmLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //sets arm to zero at the beginning of autonomous
        armliftlimit= 3000;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

/**Team marker controls
 * Driver 1
 */
/*            //For testing only
            if (gamepad1.x) {
                robot.mascot.setPosition(.0);
            }
            if (gamepad1.y) {
                robot.mascot.setPosition(0.5);
            }*/

/** Lift controls
 * Driver 1
 */
            if (gamepad1.left_bumper  ) {
                robot.LiftMotor.setPower(1);
            }
                else if (gamepad1.right_bumper && robot.lSLimit2.getState()) {
                robot.LiftMotor.setPower(-1);
            }
                    else {robot.LiftMotor.setPower(0);
            }


/** Latch controls
 * Driver 1
 */
            if (gamepad1.b) {
                robot.latch.setPosition(.5);
            }
            if (gamepad1.a) {
                robot.latch.setPosition(0);
            }


/** Turn Table
 * Driver 2
 */
            double TurnTablePow;
            TurnTablePow = Range.clip(gamepad2.right_stick_x, -1, 1);
            robot.TurnTable.setPower(TurnTablePow);

/** Arm Extension
 * Driver 2
 */
            double ArmExtendPow;
            ArmExtendPow = Range.clip(gamepad2.left_stick_y, -.4, .95);
            robot.ArmEMotor.setPower(ArmExtendPow);
/** Arm Lift
 * Driver 2
 */
        //set limit on max height
        if (gamepad2.left_bumper && robot.ArmLMotor.getCurrentPosition()<= armliftlimit) {
            robot.ArmLMotor.setPower(.8);
        }
        //prevents moving lower than switch.
        else if (gamepad2.right_bumper && robot.ArmHome.getState()) {
            robot.ArmLMotor.setPower(-0.5);
        }
        else {robot.ArmLMotor.setPower(0);
        }

/** Scoop motion
 * Driver 2
 */
//creates variables used for the servo side2side and updown motion.
        double x;
        double servoDelayTime;
        //could move to hardware
        servoDelayTime=.02;
        //skips servos unless runtime is greater than 20 ms.
        if( runtime.time() > servoDelayTime ) {

//Need to maintain power to the servo to maintain position
            x = robot.ScoopP.getPosition();
            x = Range.clip(x, 0, 1);
            if (gamepad2.dpad_up) {
                robot.ScoopP.setPosition(x + 0.01);

            } else if (gamepad2.dpad_down) {
                robot.ScoopP.setPosition(x - 0.01);
            }

//Need to maintain power to the servo to maintain position
            double y;
            y = robot.ScoopS.getPosition();
            y = Range.clip(y, 0.1, .9);
            if (gamepad2.dpad_right) {
                robot.ScoopS.setPosition(y + 0.01);

            } else if (gamepad2.dpad_left) {
                robot.ScoopS.setPosition(y - 0.01);
            }
            //resets runtime allowing for slower servo function
            runtime.reset();
        }

/** Sweeper
 * Driver 2
 */

            if (gamepad2.x) {
                robot.ScoopSweep.setPosition(1);
            }
            else if (gamepad2.y) {
                robot.ScoopSweep.setPosition(0);
            }
            else {robot.ScoopSweep.setPosition(.5);
            }

/**Main drive controls
 * Driver 1
 */

            double r = Math.hypot((double) gamepad1.left_stick_x, (double) gamepad1.left_stick_y);
            double robotAngle = Math.atan2((double) gamepad1.left_stick_y, (double) gamepad1.left_stick_x) - 0.7853981633974483;
            double rightX = (double) gamepad1.right_stick_x;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            robot.leftFDrive.setPower(v1);
            robot.rightFDrive.setPower(v2);
            robot.leftBDrive.setPower(v3);
            robot.rightBDrive.setPower(v4);
        }

    @Override
    public void stop() {
    }
}

