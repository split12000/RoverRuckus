package org.firstinspires.ftc.teamcode;
 // Program adapted and changed by Alissa Jurisch with help from our adult mentors

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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *
 *  This code ALSO requires that you have a REV Robotics hub with internal gyro with the name "imu"
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *  If your robot lands and moves around you will want to INIT again.
 *
 *  Note: all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process.
 *
 */

@Autonomous(name="Gold",group="Test")
@Disabled
public class GoldImuWebcam extends LinearOpMode {

    //Calls Robot Hardware and assigns a variable name
    Hardware2018         robot   = new Hardware2018();   // Use a Our 2018 hardware
    private ElapsedTime     runtime = new ElapsedTime();


    public double tY = 56;
    public double tX = 6;

    public double target;

    // Personal Vuforia key

    private static final String VUFORIA_KEY = " Ab8zBE//////AAABmYzZTQzGpU5Tsnyh/Ry01qRHmnySK+o+6WFiZHmhftfiAIqtSFTeTtD9cd3TpVuxtJ1YMaMEawjd/ZioW9jjAFF3sthi7POTW0tXO1ubdhU5Fsl+q1IS8ss457LoEZnGwbNzh6/oM+gCSf8yRBTUIxyVnP7WOb4trPFsmewgAiWVrhIp1aBz/4SMtPCgAbKiV4Ksecu8po2LGIT1epNcCO179kpwOnBUjyZnPwJwHQ/eo7bJZUZZ/h3SqKa436YXb/7NdVn4LZfYl50bk9T1ZngaL8XJzq37Of0Z+rOiuO4LUDkU/FPW3J3+g8hqGNQw3BCLIun3hb9KKR4JpCji/JOOeOW8X/nGSyNBSNSwIlFx ";

    /*
     Select which camera   FRONT is the correct choice for a webcam.
     Valid choices are:  BACK or FRONT
    */
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;


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


            /* Step through each leg of the path,
              Note: Reverse movement is obtained by setting a negative distance (not speed)
              Put a hold after each turn
             */


              //Step 1 Lower robot
/*

              LSLift(robot.LS_SPEED,5.7);     // Lowers lift required distance, while ensuring no damage to robot components
           robot.LiftMotor.setPower(0);

              //Step 2 Release latch

            robot.latch.setPosition(.5);

            sleep(1000);
*/

         //Step 3 move robot away from Lander to avoid scratching lander


            gyroDrive(Hardware2018.DRIVE_SPEED, 22, 0);


              //Step 4 Turn to center of field 90 degrees to avoid lander legs and minerals.

            gyroTurn(Hardware2018.TURN_SPEED, 90);
            gyroHold(Hardware2018.TURN_SPEED, 90, 0.5);


                //Step 5 move towards center of field

           gyroDrive(Hardware2018.DRIVE_SPEED, 28.0, 90);


                //Turn towards navigation target
            gyroTurn(Hardware2018.TURN_SPEED, 45);
            gyroHold(Hardware2018.TURN_SPEED, 45, 0.5);


            /**Next moves calculated by Navigation target.

            ***** Next moves based on Front and Back Walls only.
            ***** This is the Gold Autonomous.
            */
            /* This part of the program uses the Vuforia positioning on the field.
            ***** We use where we are to calculate the distance to where we want to go
            ***** Math.signum tells us if we are Red or Blue alliance
            ***** and adjusts our desired drive distance accordingly
            ***** This works well as our robot doesn't fit under the lander so we can
            ***** Not be at location (0,0) and will always return a value of 1 or -1.
            */

            webcam();    // pulls current target data from method. Returns tY (distance from origin)

            double distX= 62-Math.abs(tX);  //calculate forward travel needed to place robot close but not running into wall. (Where we want to be - where we are)
            double distY;   //  Calculate distance to depot based on where we are.
                  if (Math.signum(tX)<0) distY = 47 - tY;
                    else distY = 47 + tY;
            telemetry.addData("readings","%5.1f,%5.1f",tY,tX);
            telemetry.addData("forward", "%5.1f,%5.1f", distY,distX);
            telemetry.update();
           //sleep(5000);  // Was used for troubleshooting


            // Drive robot forward along "X-axis" based on distance from target.
             gyroDrive(Hardware2018.DRIVE_SPEED, distX, 45.0);



            //Turn to Depot
            gyroTurn(Hardware2018.TURN_SPEED, 325);
            gyroHold(Hardware2018.TURN_SPEED, 325, .5);

            //Drive to Depot
            gyroDrive(Hardware2018.DRIVE_SPEED, distY, 325);

            // Drop team mascot
             robot.mascot.setPosition(0.5);
               sleep(500);
            robot.mascot.setPosition(0);
            sleep(500);

            //Turn towards crater
           gyroTurn(Hardware2018.TURN_SPEED, 133.0);
            gyroHold(Hardware2018.TURN_SPEED, 133.0, 0.7);

            //Insert mineral sweep here

            // Drive robot to crater
            gyroDrive(Hardware2018.DRIVE_SPEED, distY + 30, 133);
            gyroHold(Hardware2018.DRIVE_SPEED,133,0.5);
            telemetry.addData("Path", "Complete");
            telemetry.update();




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
                    robot.LiftMotor.isBusy() && robot.lSLimit1.getState() && robot.lSLimit2.getState());

        }
    }


  public String webcam() {

      Hardware2018.webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraName = Hardware2018.webcamName;


        //  Instantiate the Vuforia engine
      robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.robot.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, Hardware2018.mmFTCFieldWidth, Hardware2018.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -Hardware2018.mmFTCFieldWidth, Hardware2018.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-Hardware2018.mmFTCFieldWidth, 0, Hardware2018.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(Hardware2018.mmFTCFieldWidth, 0, Hardware2018.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
        while (opModeIsActive()) {

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();  //Returns values in millimeters, must be adjusted to inches for our code.

                //Left in to use for creating
                // other programs.
                       // telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                         //   translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                tX = translation.get(0) / Hardware2018.mmPerInch; //Use for Back/Audience wall targets
                tY = translation.get(1) / Hardware2018.mmPerInch; //Use for Alliance wall targets


                return String.format(Locale.getDefault(),"translation=%.3f,%.5f",tX,tY);




                // express the rotation of the robot in degrees.
               // Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
               // telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

        }

      return String.valueOf(0);
  }


}