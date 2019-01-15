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
/*Program adapted and changed by Alissa Jurisch with help from our adult mentors
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file resets the appropriate hardware back to our starting position.
 *
 * It uses hardware class to define the drive for the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have mechanical limit switches installed to tell you when
 * you are back at "home" position.
 *
 */

@Autonomous(name="HomingRoutine",group="Test")
//@Disabled
public class HomingRoutine extends LinearOpMode {

    //Calls Robot Hardware and assigns a variable name
    Hardware2018 robot = new Hardware2018();   // Use our 2018 hardware
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //since hardware class sets motors to run with Encoder we have change the
        //run mode.
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        if (opModeIsActive()) {

            //Set servos back to home position

            /** Set latch back to home*/
            robot.latch.setPosition(0);
            robot.mascot.setPosition(0);

            sleep(1000);

            /**Brings down lift until the limit switch is hit */
            robot.LiftMotor.setPower(-.4);
            while (robot.lSLimit2.getState()) {
                sleep(1);
            }
            robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           // robot.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LiftMotor.setTargetPosition(120);
            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /**
             * Added arm lift and reset extension to zero.
             */
            //Moves scoop out of the way so that it does not hit the ground while the arm is homing.
            robot.ScoopP.setPosition(1);
            sleep(250);
            robot.ArmLMotor.setPower(-.6);
            while (robot.ArmHome.getState()){
                sleep(1);
            }
            robot.ArmLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            /** Bring arm back up to level*/
            robot.ArmLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.ArmLMotor.setTargetPosition(2500);
            robot.ArmLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmLMotor.setPower(.6);
            while (opModeIsActive()&&(robot.ArmLMotor.isBusy()));


            /**
             * Turntable home position
             * Run until limit hit then turn back until at desired location
             *//*
            robot.TurnTable.setPower(-.6);
            while (robot.TTLimit1.getState()){
                sleep(1);
            }
            robot.TurnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           // robot.TurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.TurnTable.setTargetPosition(give me number);
            //robot.TurnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.TurnTable.setPower(.6);
            while (opModeIsActive() && (robot.TurnTable.isBusy()));
            */

//used for determining starting position of home routine
            double armposition = robot.ArmLMotor.getCurrentPosition();
            double liftposition = robot.LiftMotor.getCurrentPosition();
            double turntable = robot.TurnTable.getCurrentPosition();
            double armextension = robot.ArmEMotor.getCurrentPosition();
            telemetry.addData("Arm position","%5.2f",armposition);
            telemetry.addData("Lift position","%5.2f",liftposition);
            telemetry.addData("Turntable position","%5.2f",turntable);
            telemetry.addData("Arm extension position","%5.2f",armextension);
            telemetry.update();
            //sleep(10000);


            /** Bring up scoop and then bring back onto robot*/
            robot.ScoopP.setPosition(.5);
            sleep(250);
            robot.ScoopS.setPosition(1);
            sleep(500);

        }
    }
}

/**Rev 1 ready for Week 0 meets.  Homes lift and latch. No arm attached.
 * Rev 2 ready for meet in January.  Added Scoop and arm homing.
 * Rev 3 added Turntable homing.
 * Need limit switches installed to home turntable.
 * Need retraction mechanism installed to retract arm
 */


