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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower and upper case and some may have single spaces between words.
 *
 * Motor channel:   Left front motor:       "lfm"
 * Motor channel:   Right front motor:      "rfm"
 * Motor channel:   Left back motor:        "lbm"
 * Motor channel:   Right back motor:       "rbm"
 *
 * Motor channel:   Lift motor:             "updownmotor"
 * Servo channel:   latch:                  "latch"
 * Digital channel: LS lower limit switch:  "LSLimit1"
 * Digital channel: LS upper limit switch:  "LSLimit2"
 *
 * Motor channel:  Turn table motor:        "TurnTable"
 * Motor channel:  Arm lift motor:          "ArmLMotor"
 * Motor channel:  Arm extend motor:        "ArmEMotor"
 * Servo channel:  Scoop position motor:    "ScoopP"
 * Servo channel:  Scoop sweep motor:       "ScoopS"
 * Servo channel:  Scoop sweep motor:       "ScoopSweep"
 * Digital channel: Turntable right switch  "TTLimit1"
 * Digital channel: Turntable left switch   "TTLimit2"
 * Digital channel: Arm home limit switch   "ArmHome"
 *
 * Servo channel:  Team mascot servo        "mascot"
 *
 * I2C channel:  imu                        "imu"
 *
 * Webcam                                   "webcam"
 */
public class Hardware2018
{

    //Mecanum motors
    public DcMotor leftFDrive;
    public DcMotor rightFDrive;
    public DcMotor leftBDrive;
    public DcMotor rightBDrive;

    //Drop motor and limits
    public DcMotor LiftMotor;
    public DigitalChannel lSLimit1;
    public DigitalChannel lSLimit2;
    public Servo latch;

    //Arm motors and etc....
    public DcMotor TurnTable;
    public DcMotor ArmLMotor;
    public DcMotor ArmEMotor;
    public Servo ScoopP;
    public Servo ScoopS;
    public Servo ScoopSweep;
    public DigitalChannel TTLimit1;
    public DigitalChannel TTLimit2;
    public DigitalChannel ArmHome;

    //Team Marker now uses arm scoop servos
    public Servo mascot;


    //IMU
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();


    //Initialize Camera variables.
    static WebcamName webcamName;
    VuforiaLocalizer vuforia;




    // Motor Parameters
    public static final double COUNTS_PER_MOTOR_REV_40 = 1120;  //REV HD 40:1 motor information
    public static final double COUNTS_PER_MOTOR_REV_20 = 620;     //REV HD 20:1 motor information
    public static final double DRIVE_GEAR_REDUCTION = 1;  //This is <1.0 if geared up  1.3 starting point for REV HD motor
    public static final double WHEEL_DIAMETER_INCHES = 4.0;  //This is for figuring circumference
    public static final double LIFT_REDUCTION = 1.0;   //This is the fudge factor for lift Rev 40 motor. Was 0.8 for Rev 20
    public static final double LS_PITCH_LIFT = 3.3;   //Pitch of lead screw lifting arm
    public static final double LS_GEAR_RATIO = 0.5;    //Motor Gear teeth / Lead Screw Gear teeth
    public static final double DROP_DISTANCE =6.5;  //Distance to lower robot

    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_40 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double REVS_PER_INCH_LIFT = (COUNTS_PER_MOTOR_REV_40 * LS_GEAR_RATIO * LS_PITCH_LIFT *LIFT_REDUCTION);

    public static final double HEADING_THRESHOLD = 1;      // Fudge factor. As tight as we can make it with gyro
    public static final double P_TURN_COEFF = 0.1;     // Larger is less stable
    public static final double P_DRIVE_COEFF = 0.1;     // Larger is less stable

    public static final double DRIVE_SPEED = 0.75;
    public static final double TURN_SPEED = 0.25;  //Changed not tested
    public static final double TURN_SPEED_LARGE = .25;
    public static final double TURN_SPEED_SMALL = .15;
    public static final double LS_SPEED = .9;


    // Field Parameters for Camera
    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor


    // local OpMode members.
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Constructor
    public Hardware2018(){

    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        leftFDrive = hwMap.dcMotor.get("lfm"); //Hub 2 Motor 3
        rightFDrive = hwMap.dcMotor.get("rfm"); //Hub 2 Motor 2
        leftBDrive = hwMap.dcMotor.get("lbm"); //Hub 2 Motor 1
        rightBDrive = hwMap.dcMotor.get("rbm"); //Hub 2 Motor 0

        // Initialize the Landing mechanism variables.

        LiftMotor = hwMap.dcMotor.get("updownmotor"); //Hub 3 Motor 0
        lSLimit1 = hwMap.digitalChannel.get("LSLimitUP"); //Hub 2 Digital 1
        lSLimit2 = hwMap.digitalChannel.get("LSLimitDOWN"); //Hub 2 Digital 3
        latch = hwMap.servo.get("latch"); //Hub 2 Servo 0

        // Initialize the Arm
        TurnTable = hwMap.dcMotor.get("TurnTable"); //Hub 3 Motor 2
        ArmLMotor = hwMap.dcMotor.get("ArmLMotor"); //Hub 3 Motor 3
        ArmEMotor = hwMap.dcMotor.get("ArmEMotor"); //Hub 3 Motor 1
        ScoopP = hwMap.servo.get("ScoopP"); //Hub 3 Servo 1 Orange
        ScoopS = hwMap.servo.get("ScoopS"); //Hub 3 Servo 2 Nothing
        ScoopSweep = hwMap.servo.get("ScoopSweep"); //Hub 3 Servo 3 Black
        TTLimit1 = hwMap.digitalChannel.get("TTLimit1"); //Hub
        TTLimit2 = hwMap.digitalChannel.get("TTLimit2"); //Hub
        ArmHome = hwMap.digitalChannel.get("ArmHome"); //Hub 3 Digital 1

        // Intialize the servo for dropping team mascot
        mascot = hwMap.servo.get("mascot"); //Hub 2 Servo 5

         // name our front Webcam.
        webcamName = hwMap.get(WebcamName.class, "Webcam");

         //Set reverse motor direction where required.
        leftFDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFDrive.setDirection(DcMotor.Direction.REVERSE);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        TurnTable.setDirection(DcMotor.Direction.FORWARD);
        ArmEMotor.setDirection(DcMotor.Direction.FORWARD);
        ArmLMotor.setDirection(DcMotor.Direction.FORWARD);

        ArmHome.setMode(DigitalChannel.Mode.INPUT);
        lSLimit1.setMode(DigitalChannel.Mode.INPUT);
        lSLimit2.setMode(DigitalChannel.Mode.INPUT);
    }
 }

