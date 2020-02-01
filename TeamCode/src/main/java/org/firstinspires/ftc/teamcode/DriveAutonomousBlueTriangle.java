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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DriveAutonomousBlueTriangle", group="Linear Opmode")
//@Disabled
public class DriveAutonomousBlueTriangle extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armLyft = null;
    private Servo tailGateServo = null;
    double tailGateServoSpeed = 0.5;
    private Servo armServo = null;
    double armServoSpeed = 0.5;
    DistanceSensor sensorDistance;
    private DistanceSensor sensorRange;


    public void setDrive(double drivePower, double turnPower) {
        double leftPower;
        double rightPower;
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -drivePower;
        double turn  =  turnPower;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armLyft  = hardwareMap.get(DcMotor.class, "arm_lift");
        tailGateServo = hardwareMap.servo.get("tail_gate_servo");
        armServo = hardwareMap.servo.get("arm_servo");
        // 2m distance sensor
        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armLyft.setDirection(DcMotor.Direction.FORWARD);
        tailGateServo.setPosition(tailGateServoSpeed);
        armServo.setPosition(armServoSpeed);
        //servo.setPosition(servoPosition);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Drive to CZ
        setDrive(-1.0,0.0);
        sleep(950);
        setDrive(0.0,0.0);

        // Grab CZ
        tailGateServo.setPosition(0.0);
        sleep(850);
        tailGateServo.setPosition(0.5);

        // Move CZ Back
        setDrive(1.0,0.0);
        sleep(1150);
        setDrive(0.0,0.0);

        // Let Go of CZ
        tailGateServo.setPosition(1.0);
        sleep(800);
        tailGateServo.setPosition(0.5);

        // Turn to Face Right
        setDrive(0.0, -1.0);
        sleep(3000);
        setDrive(0.0, 0.0);

        // Move Forward
        setDrive(-1.0, 0.0);
        sleep(1050);
        setDrive(0.0,0.0);

        // Turn Left
        setDrive(0.0, 1.0);
        sleep(2300);
        setDrive(0.0,0.0);

        // Move Forward
        setDrive(-1.0,0.0);
        sleep(1500);
        setDrive(0.0,0.0);

        // Turn Left
        setDrive(0.0, 1.0);
        sleep(4600);
        setDrive(0.0, 0.0);

        // Move Forward
        setDrive(-1.0,0.0);
        sleep(800);
        setDrive(0.0,0.0);

        // Turn Left to Face CZ
        setDrive(0.0,1.0);
        sleep(4600);
        setDrive(0.0,0.0);

        // Push CZ Into Zone
        setDrive(-1.0,0.0);
        sleep(2100);
        setDrive(0.0,0.0);

        // Move Back a 'Lil
        setDrive(1.0,0.0);
        sleep(500);
        setDrive(0.0,0.0);

        // Turn Left
        setDrive(0.0,1.0);
        sleep(2100);
        setDrive(0.0,0.0);

        // Bring down arm
        armLyft.setPower(1.0);
        sleep(100);
        armLyft.setPower(0.0);

        // Move Forward
        setDrive(-1.0,0.0);
        sleep(1850);
        setDrive(0.0,0.0);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        //telemetry.addData("Arm power", "(%.2f)", armPower);
        telemetry.update();
    }
}
