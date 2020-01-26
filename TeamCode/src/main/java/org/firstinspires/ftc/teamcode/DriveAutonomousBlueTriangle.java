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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

        // Drive straight for 2.5 seconds.
        double drivePower = 1.0;
        double turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(2500);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        // Turn for 2.5 seconds.
        drivePower = 0.0;
        turnPower = 1.0;
        setDrive(drivePower, turnPower);
        sleep(2500);
        turnPower = 0.0;
        setDrive(drivePower, turnPower);

        // Drive straight for 0.5 seconds.
        drivePower = 1.0;
        turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(500);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        // Turn for 2 seconds.
        drivePower = 0.0;
        turnPower = 1.0;
        setDrive(drivePower, turnPower);
        sleep(2000);
        turnPower = 0.0;
        setDrive(drivePower, turnPower);

        // Drive straight for 2 seconds.
        drivePower = 1.0;
        turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(2000);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        // Drive backwards for 0.5 seconds.
        drivePower = -1.0;
        turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(500);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        // Drive straight for 2 seconds.
        drivePower = 1.0;
        turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(2000);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        // Drive backwards for 1 seconds.
        drivePower = -1.0;
        turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(500);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        // Drive straight for 2 seconds.
        drivePower = 1.0;
        turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(2000);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        // Drive backwards for .5 seconds.
        drivePower = -1.0;
        turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(500);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        // Drive straight for 2 seconds.
        drivePower = 1.0;
        turnPower = 0.0;
        setDrive(drivePower, turnPower);
        sleep(2000);
        drivePower = 0.0;
        setDrive(drivePower, turnPower);

        /*
            // Arm forward is y up. Arm backward is y down.
            double armPower = gamepad1.right_stick_y;
            armLyft.setPower(armPower);


            // Hand/Grip. Pressing A closes it. Pressing B opens it.
            if (gamepad1.y == true) {
                tailGateServoSpeed = 1.0;
                tailGateServo.setPosition(tailGateServoSpeed);
                sleep(310);
                tailGateServoSpeed = 0.5;
                tailGateServo.setPosition(tailGateServoSpeed);
            }
            if (gamepad1.x == true) {
                tailGateServoSpeed = 0.0;
                tailGateServo.setPosition(tailGateServoSpeed);
                sleep(300);
                tailGateServoSpeed = 0.5;
                tailGateServo.setPosition(tailGateServoSpeed);
            }

            if(gamepad1.a == true) {
                armServo.setPosition(1.0);
                sleep(1000);
                armServo.setPosition(0.5);
            }

            if(gamepad1.b == true) {
                armServo.setPosition(0.0);
                sleep(1000);
                armServo.setPosition(0.5);
            }

         */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        //telemetry.addData("Arm power", "(%.2f)", armPower);
        telemetry.update();
    }
}