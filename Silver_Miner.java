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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses hardware class to define the drive motors on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *
 * This code ALSO requires that you have a REV Robotics hub with internal gyro with the name "imu"
 *
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  The REV Expansion hub must be in a horizontal position for this code.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *  If your robot lands and moves around you may want to INIT again.
 *
 *  Note: all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process.
 *
 */

@Autonomous(name="Silver_Miner",group="Test")
//@Disabled
public class Silver_Miner extends LinearOpMode {

    //Calls Robot Hardware and assigns a variable name
    Hardware2018 robot = new Hardware2018();   // Use a Our 2018 hardware
    private ElapsedTime runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // Personal Vuforia key
    private static final String VUFORIA_KEY = " Ab8zBE//////AAABmYzZTQzGpU5Tsnyh/Ry01qRHmnySK+o+6WFiZHmhftfiAIqtSFTeTtD9cd3TpVuxtJ1YMaMEawjd/ZioW9jjAFF3sthi7POTW0tXO1ubdhU5Fsl+q1IS8ss457LoEZnGwbNzh6/oM+gCSf8yRBTUIxyVnP7WOb4trPFsmewgAiWVrhIp1aBz/4SMtPCgAbKiV4Ksecu8po2LGIT1epNcCO179kpwOnBUjyZnPwJwHQ/eo7bJZUZZ/h3SqKa436YXb/7NdVn4LZfYl50bk9T1ZngaL8XJzq37Of0Z+rOiuO4LUDkU/FPW3J3+g8hqGNQw3BCLIun3hb9KKR4JpCji/JOOeOW8X/nGSyNBSNSwIlFx ";

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**
     Select which camera   FRONT is the correct choice for a webcam.
     Valid choices are:  BACK or FRONT
     */
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    //These lines are used if we go back to reading the targets.
   /* private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
*/

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Initialize Rev internal IMU variables

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //Ensure the robot it stationary, then reset the encoders and calibrate the gyro.

        robot.leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Let driver know what we are calibrating;

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        /* Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
          on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
          and named "imu".
         */
        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Where the IMU is calibrated. May copy to after robot placed on field if having issues with repeatability.

        robot.imu.initialize(parameters);

        // Tells Driver that IMU is calibrating.

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(5);
            idle();

            telemetry.addData(">", "Robot Ready.");    //
            telemetry.update();

            //Wait for the game to start (driver presses PLAY)
            waitForStart();

            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /** Step through each leg of the path.
                Note: Reverse movement is obtained by setting a negative distance (not speed)
                Put a hold after each turn.
             */

            //Step 1 Lower robot
            LSLift(Hardware2018.LS_SPEED, robot.DROP_DISTANCE);     // Lowers lift required distance, while ensuring no damage to robot components
            robot.LiftMotor.setPower(0);


            //Step 2 Release latch and initialize webcam
            robot.latch.setPosition(.5);

            initVuforia();
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
            sleep(1000);

            //Step 3 move robot away from Lander to avoid scratching lander
            gyroDrive(Hardware2018.DRIVE_SPEED, 2, 0);

            //Step 4 turns the robot to guarantee the two minerals we see are on the left
            gyroTurn(Hardware2018.TURN_SPEED, 10);
            gyroHold(Hardware2018.TURN_SPEED, 10, .4);

            //Step 5 Determines which position the mineral is in and determines path from there.
            mineralD();

            //prevents looping issue we were seeing at times.
            stop();
        }
    }

    //Insert what this Method does
            public void mineralD() {
                /** Activate Tensor Flow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                }

                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 2) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;

                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                                                        }
                                }
                                if (goldMineralX != -1 || silverMineral1X != -1 ) {
                                    if (goldMineralX == -1) {  //This checks to see if Gold Mineral was seen, if not it is in right position
                                        telemetry.addData("Gold Mineral Position", "Right");
                                        //shutdown mineral detection
                                        tfod.shutdown();
                                        //turn and drive through mineral into crater
                                        gyroTurn(Hardware2018.TURN_SPEED, -30);
                                        gyroDrive(Hardware2018.DRIVE_SPEED, 34, -30);
                                        //turn off all motors
                                        robot.leftBDrive.setPower(0);
                                        robot.leftFDrive.setPower(0);
                                        robot.rightBDrive.setPower(0);
                                        robot.rightFDrive.setPower(0);
                                        // exits out of Method
                                        break;
                                    } else if (goldMineralX < silverMineral1X) { // Is Gold Mineral in Left position
                                        telemetry.addData("Gold Mineral Position", "Left");
                                        //shutdown mineral detection
                                        tfod.shutdown();
                                        //turn and drive through mineral into crater
                                        gyroTurn(Hardware2018.TURN_SPEED, 30);
                                        gyroDrive(Hardware2018.DRIVE_SPEED, 45, 30);
                                        //turn off all motors
                                        robot.leftBDrive.setPower(0);
                                        robot.leftFDrive.setPower(0);
                                        robot.rightBDrive.setPower(0);
                                        robot.rightFDrive.setPower(0);
                                        // exits out of Method
                                        break;
                                    } else { //Gold Mineral in Center position and/or continue as though it is.
                                        telemetry.addData("Gold Mineral Position", "Center");
                                        //shutdown mineral detection
                                        tfod.shutdown();
                                        //turn and drive through mineral into crater
                                        gyroTurn(Hardware2018.TURN_SPEED, 0);
                                        gyroDrive(Hardware2018.DRIVE_SPEED, 34, 0);
                                        //turn off all motors
                                        robot.leftBDrive.setPower(0);
                                        robot.leftFDrive.setPower(0);
                                        robot.rightBDrive.setPower(0);
                                        robot.rightFDrive.setPower(0);
                                        // exits out of Method
                                        break;
                                    }

                                }

                            }

                        }
                    }
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
            moveCounts = (int) (distance * Hardware2018.COUNTS_PER_INCH);

            newLeftFTarget = robot.leftFDrive.getCurrentPosition() + moveCounts;
            newRightFTarget = robot.rightFDrive.getCurrentPosition() + moveCounts;
            newLeftBTarget = robot.leftBDrive.getCurrentPosition() + moveCounts;
            newRightBTarget = robot.rightBDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFDrive.setTargetPosition(newLeftFTarget);
            robot.rightFDrive.setTargetPosition(newRightFTarget);
            robot.leftBDrive.setTargetPosition(newLeftBTarget);
            robot.rightBDrive.setTargetPosition(newRightBTarget);

            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 0.05);
            robot.leftFDrive.setPower(speed);
            robot.rightFDrive.setPower(speed);
            robot.leftBDrive.setPower(speed);
            robot.rightBDrive.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (robot.leftFDrive.isBusy() && robot.rightFDrive.isBusy() && robot.leftBDrive.isBusy() && robot.rightBDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, Hardware2018.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 0.05) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftFDrive.setPower(leftSpeed);
                robot.rightFDrive.setPower(rightSpeed);
                robot.rightBDrive.setPower(leftSpeed);
                robot.leftBDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftFTarget, newRightFTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftFDrive.getCurrentPosition(),
                        robot.rightFDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFDrive.setPower(0);
            robot.rightFDrive.setPower(0);
            robot.leftBDrive.setPower(0);
            robot.rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        while (opModeIsActive() && !onHeading(speed, angle, Hardware2018.P_TURN_COEFF)) {
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
            onHeading(speed, angle, Hardware2018.P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFDrive.setPower(0);
        robot.rightFDrive.setPower(0);
        robot.leftBDrive.setPower(0);
        robot.rightBDrive.setPower(0);
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

        if (Math.abs(error) <= Hardware2018.HEADING_THRESHOLD) {
            steer = 0.0;
            leftFSpeed = 0.0;
            rightFSpeed = 0.0;
            leftBSpeed = 0.0;
            rightBSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
           /*
            rightFSpeed  = speed * Math.cos(steer);
            leftFSpeed   = -rightFSpeed;
            rightBSpeed = -rightFSpeed;
            leftBSpeed = rightFSpeed;
            */

            leftFSpeed = speed * Math.cos(error) + steer / 5;
            rightFSpeed = speed * Math.sin(error) - steer / 5;
            leftBSpeed = speed * Math.sin(error) + steer / 5;
            rightBSpeed = speed * Math.cos(error) - steer / 5;

        }

        // Send desired speeds to motors.
        robot.leftFDrive.setPower(leftFSpeed);
        robot.rightFDrive.setPower(rightFSpeed);
        robot.leftBDrive.setPower(leftBSpeed);
        robot.rightBDrive.setPower(rightBSpeed);

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
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

    /**Runs lift motor a desired distance to allow unlatching from lander
     *
     * @param speed  How fast we run the motor
     * @param distance  How far you want the lift to move
     */

    public void LSLift(double speed,
                       double distance) {

        int newLSTarget;
        int moveCounts;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * Hardware2018.REVS_PER_INCH_LIFT);
            newLSTarget = robot.LiftMotor.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            robot.LiftMotor.setTargetPosition(newLSTarget);


            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed= Range.clip(Math.abs(speed), 0.0, 1);
            robot.LiftMotor.setPower(speed);

            //Next step checks to make sure that we have not hit our limit switches.
            //This prevents damage to the robot.

            while (opModeIsActive() &&
                    robot.LiftMotor.isBusy());

        }
    }

    /**initialize Vuforia for tfod
     */

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load trackables here is using targets as well.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}