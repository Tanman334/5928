    /*
    Copyright (c) 2016 Robert Atkinson

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
    are permitted (subject to the limitations in the disclaimer below) provided that
    the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

    Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or
    other materials provided with the distribution.

    Neither the name of Robert Atkinson nor the names of his contributors may be used to
    endorse or promote products derived from this software without specific prior
    written permission.

    NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
    LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
    TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
    THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    */
    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.util.ElapsedTime;

    /*
     * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
     * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
     * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
     * It includes all the skeletal structure that all linear OpModes contain.
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @Autonomous(name="Auto", group="Linear Opmode")  // @Autonomous(...) is the other common choice
    //Version: Asparagus
    public class Autonomous5928 extends LinearOpMode {

        /* Declare OpMode members. */
        private ElapsedTime runtime = new ElapsedTime();
        private ElapsedTime time;
        // DcMotor leftMotor = null;
        // DcMotor rightMotor = null;

        private DcMotor frontLeft = null;
        private DcMotor frontRight = null;
        private DcMotor backLeft = null;
        private DcMotor backRight = null;

        private DcMotor frontLift = null;
        private DcMotor backLift = null;
        //both = illogical


        private void go(int direction) {
            if (direction == 1) {
                // forward
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.REVERSE);

                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setDirection(DcMotor.Direction.FORWARD);
            }
            else if(direction == 2){
                // forward and right
                frontLeft.setDirection(DcMotor.Direction.REVERSE);

                backRight.setDirection(DcMotor.Direction.FORWARD);
            }
            else if(direction == 3){
                // right
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.FORWARD);

                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setDirection(DcMotor.Direction.FORWARD);
            }
            else if(direction == 4){
                // back and right
                backLeft.setDirection(DcMotor.Direction.FORWARD);

                frontRight.setDirection(DcMotor.Direction.REVERSE);
            }
            else if(direction == 5){
                // back
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setDirection(DcMotor.Direction.FORWARD);

                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setDirection(DcMotor.Direction.REVERSE);
            }
            else if(direction == 6){
                // back and left
                frontLeft.setDirection(DcMotor.Direction.FORWARD);

                backRight.setDirection(DcMotor.Direction.REVERSE);
            }
            else if(direction == 7){
                // left
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setDirection(DcMotor.Direction.REVERSE);

                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setDirection(DcMotor.Direction.REVERSE);
            }
            else if(direction == 8){
                // forward and left
                backLeft.setDirection(DcMotor.Direction.REVERSE);

                frontRight.setDirection(DcMotor.Direction.FORWARD);
            }
            else if(direction == 9){
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.REVERSE);

                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setDirection(DcMotor.Direction.REVERSE);
            }
            else if(direction == 10){
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setDirection(DcMotor.Direction.FORWARD);

                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setDirection(DcMotor.Direction.FORWARD);
            }
            else{
                // In case Tanner forgets how to count
                telemetry.addData("Error", "Invalid Direction");
                telemetry.update();
            }
        }

        private void speed(double fLSpeed, double bLSpeed, double fRSpeed,double bRSpeed){
            frontLeft.setPower(fLSpeed);
            backLeft.setPower(bLSpeed);

            frontRight.setPower(fRSpeed);
            backRight.setPower(bRSpeed);
        }

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            /* eg: Initialize the hardware variables. Note that the strings used here as parameters
             * to 'get' must correspond to the names assigned during the robot configuration
             * step (using the FTC Robot Controller app on the phone).
             */
            // leftMotor  = hardwareMap.dcMotor.get("left_drive");
            // rightMotor = hardwareMap.dcMotor.get("right_drive");

            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            backLeft = hardwareMap.dcMotor.get("backLeft");
            backRight = hardwareMap.dcMotor.get("backRight");

            frontLift = hardwareMap.dcMotor.get("frontLift");
            backLift = hardwareMap.dcMotor.get("backLift");

            time = new ElapsedTime();

            // eg: Set the drive motor directions:
            // "Reverse" the motor that runs backwards when connected directly to the battery
            // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


            // Wait for the game to start (driver presses PLAY)

            runtime.reset();

            double timer = time.seconds();
            time.reset();

            while(timer < 5){
                frontLift.setDirection(DcMotor.Direction.FORWARD);
                backLift.setDirection(DcMotor.Direction.REVERSE);

                frontLift.setPower(1);
                backLift.setPower(1);
            }
            //lift running

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();


                // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
                // leftMotor.setPower(-gamepad1.left_stick_y);
                // rightMotor.setPower(-gamepad1.right_stick_y);
            }
        }
    }
