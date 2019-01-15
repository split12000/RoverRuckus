package org.firstinspires.ftc.teamcode;
 /* Program adapted and changed by Alissa Jurisch with help from our adult mentors*/

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


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file was designed to test our Mecanum drive train for reliability after we had skidding on
 * the ending of our drive program as one wheel was not turning off at the same time as the other
 * three.
 *
 * This file verifies that we have matched motors and encoders. This can also be used to troubleshoot
 * loose encoder wires.
 *
 * First: Run this program with the robot on a lift for a set distance and verify that all motors stop at
 * the same time.  Once this is working proceed to step 2.
 *
 * Second: Run this program for a distance of at least 8 feet marking start point and measure distance
 * robot traveled.  If robot traveled a distance different from the requested distance.  Verify
 * the correct counts per revolution is entered.  If good then calculate the  * change needed to
 * the drive gear reduction.
 * (Distance traveled/Distance requested)* Drive_Gear_Reduction
 * Enter the corrected value and verify the robot now drives required distance.
 *
 * Suggest running step 2 a couple of times and averaging the results.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="EncoderTest",group="Test")
//@Disabled
public class EncoderTest extends LinearOpMode {

    //Mecanum motors
    private DcMotor leftFDrive;
    private DcMotor rightFDrive;
    private DcMotor leftBDrive;
    private DcMotor rightBDrive;

    //Drop motor and limits
    private DcMotor LiftMotor;
    private DigitalChannel LSLimit1;


    //IMU
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();


    // Motor Parameters
    static final double COUNTS_PER_MOTOR_REV_40 = 1120;  //REV HD 40:1 motor information
    static final double COUNTS_PER_MOTOR_REV_20 = 560;     //REV HD 20:1 motor information
    static final double DRIVE_GEAR_REDUCTION = 1;  //This is <1.0 if geared up  1.3 starting point for REV HD motor
    static final double WHEEL_DIAMETER_INCHES = 4.0;  //This is for figuring circumference
    static final double LS_PITCH_LIFT = 3.3;   //Pitch of lead screw lifting arm


    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_40 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double REVS_PER_INCH_LIFT = (COUNTS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION * LS_PITCH_LIFT);

    static final double HEADING_THRESHOLD = 1;      // Fudge factor. As tight as we can make it with gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is less stable
    static final double P_DRIVE_COEFF = 0.1;     // Larger is less stable

    static final double DRIVE_SPEED = 0.7;
    static final double TURN_SPEED = 0.02;


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         */

        leftFDrive = hardwareMap.dcMotor.get("lfm");
        rightFDrive = hardwareMap.dcMotor.get("rfm");
        leftBDrive = hardwareMap.dcMotor.get("lbm");
        rightBDrive = hardwareMap.dcMotor.get("rbm");
        /*
         * Set reverse motor direction where required.
         */
        leftBDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Initialize Rev internal IMU variables
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        /*
         *Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
         */
        leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         *Let driver know what we are calibrating;
         */
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        /* Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
         * on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
         * and named "imu".
         */
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /*
         *Where the IMU is calibrated. May move to after robot placed on field if having issues with repeatability.
         */
        imu.initialize(parameters);

        /*
         * Tells Driver that IMU is calibrating.
         */
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();


        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(5);
            idle();


            telemetry.addData(">", "Robot Ready.");    //
            telemetry.update();

             /*
              *Wait for the game to start (driver presses PLAY)
              */

            waitForStart();

            leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /**
             * Recommend driving at least 96 inches and verifying distance traveled
             *  angle should be set to zero in this program
             */
            gyroDrive(DRIVE_SPEED, 48, 0);


            // Stop all motion;
            leftFDrive.setPower(0);
            rightFDrive.setPower(0);
            leftBDrive.setPower(0);
            rightBDrive.setPower(0);
//break;
        }
    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);

            newLeftFTarget = leftFDrive.getCurrentPosition() + moveCounts;
            newRightFTarget = rightFDrive.getCurrentPosition() + moveCounts;
            newLeftBTarget = leftBDrive.getCurrentPosition() + moveCounts;
            newRightBTarget = rightBDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftFDrive.setTargetPosition(newLeftFTarget);
            rightFDrive.setTargetPosition(newRightFTarget);
            leftBDrive.setTargetPosition(newLeftBTarget);
            rightBDrive.setTargetPosition(newRightBTarget);

            leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, .2);
            leftFDrive.setPower(speed);
            rightFDrive.setPower(speed);
            leftBDrive.setPower(speed);
            rightBDrive.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    leftFDrive.isBusy() && rightFDrive.isBusy() && leftBDrive.isBusy() && rightBDrive.isBusy()){

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftFDrive.setPower(leftSpeed);
                rightFDrive.setPower(rightSpeed);
                rightBDrive.setPower(leftSpeed);
                leftBDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftFTarget, newRightFTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("Actual", "%7d:%7d", leftFDrive.getCurrentPosition(),
                        rightFDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftFDrive.setPower(0);
            rightFDrive.setPower(0);
            leftBDrive.setPower(0);
            rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftFDrive.setPower(0);
        rightFDrive.setPower(0);
        leftBDrive.setPower(0);
        rightBDrive.setPower(0);
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftFSpeed;
        double rightFSpeed;
        double leftBSpeed;
        double rightBSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftFSpeed = 0.0;
            rightFSpeed = 0.0;
            leftBSpeed = 0.0;
            rightBSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
           /* rightFSpeed  = speed * Math.cos(steer);
            leftFSpeed   = -rightFSpeed;
            rightBSpeed = -rightFSpeed;
            leftBSpeed = rightFSpeed;*/

            leftFSpeed = speed * Math.cos(error) + steer / 5;
            rightFSpeed = speed * Math.sin(error) - steer / 5;
            leftBSpeed = speed * Math.sin(error) + steer / 5;
            rightBSpeed = speed * Math.cos(error) - steer / 5;

        }

        // Send desired speeds to motors.
        leftFDrive.setPower(leftFSpeed);
        rightFDrive.setPower(rightFSpeed);
        leftBDrive.setPower(leftBSpeed);
        rightBDrive.setPower(rightBSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftFSpeed, rightFSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //above line returns an Orientation object. Adding first angle allows for Z angle to be subtracted as itself.
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void LSLift(double speed,
                       double distance) {

        int newLSTarget;
        int moveCounts;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * REVS_PER_INCH_LIFT);
            newLSTarget = LiftMotor.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            LiftMotor.setTargetPosition(newLSTarget);


            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed= Range.clip(Math.abs(speed), 0.0, 1);
            LiftMotor.setPower(speed);

            while (opModeIsActive() &&
                    LiftMotor.isBusy() && LSLimit1.getState());



        }
    }






}