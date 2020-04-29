package org.firstinspires.ftc.teamcode.CedarCityAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "GrabStone Blue", group = "LinearOpMode")
@Disabled
public class GrabStone extends LinearOpMode{

        ElapsedTime runtime = new ElapsedTime();
        private DcMotor FL;                              //declare OpMode mechanical members and their names used in the program
        private DcMotor FR;
        private DcMotor BL;
        private DcMotor BR;
        private DcMotor IL;
        private DcMotor IR;
        private DcMotor LiR;
        private DcMotor LiL;
        private Servo drag;
        private double counts_to_inches;

    @Override
        public void runOpMode()
        {
            //name motors
            counts_to_inches = Math.PI * 3.5;
            FL = hardwareMap.get(DcMotor.class, "FL");
            FR = hardwareMap.get(DcMotor.class, "FR");
            BR = hardwareMap.get(DcMotor.class, "BR");
            BL = hardwareMap.get(DcMotor.class, "BL");
            IL = hardwareMap.get(DcMotor.class, "IL");
            IR = hardwareMap.get(DcMotor.class, "IR");
            //configure motors
            BR.setDirection(DcMotor.Direction.FORWARD);
            BL.setDirection(DcMotor.Direction.REVERSE);
            FL.setDirection(DcMotor.Direction.REVERSE);
            FR.setDirection(DcMotor.Direction.FORWARD);

            //wait for program to start

            waitForStart();
            runtime.reset();

            IR.setPower(1.0);
            IL.setPower(-1.0);

            driveForwardEncoders(0.5, 2000);
            sleep(2000);
            driveBackwardEncoders(0.3, -1800);
            turnRightEncoders(0.5, 650);
            driveForwardEncoders(0.5, 2000);
            IR.setPower(-1.0);
            IL.setPower(1.0);
            sleep(1000);
            driveBackwardEncoders(0.5, -1800);



        }

        public void driveForward(double power)
        {
            BR.setPower(power);
            BL.setPower(power);
            FR.setPower(power);
            FL.setPower(power);
        }

        public void driveForwardEncoders(double power, int distance)
        {
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BR.setTargetPosition(distance);
            BL.setTargetPosition(distance);
            FR.setTargetPosition(distance);
            FL.setTargetPosition(distance);

            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy())
            {
                driveForward(power);
            }
        }

        public void driveBackward(double power)
        {
            BR.setPower(-power);
            BL.setPower(-power);
            FL.setPower(-power);
            FR.setPower(-power);
        }

        public void driveBackwardEncoders(double power, int distance)
        {
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BR.setTargetPosition(distance);
            BL.setTargetPosition(distance);
            FR.setTargetPosition(distance);
            FL.setTargetPosition(distance);

            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
            {
                driveBackward(power);
            }
        }

        public void turnRight(double power)
        {
            BR.setPower(-power);
            BL.setPower(power);
            FR.setPower(-power);
            FL.setPower(power);
        }

        public void turnRightEncoders(double power, int distance)
        {
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BR.setTargetPosition(-distance);
            BL.setTargetPosition(distance);
            FR.setTargetPosition(-distance);
            FL.setTargetPosition(distance);

            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
            {
                turnRight(power);
            }
        }

        public void turnLeft(double power)
        {
            BR.setPower(power);
            BL.setPower(-power);
            FR.setPower(power);
            FL.setPower(-power);
        }

        public void turnLeftEncoders(double power, int distance)
        {
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BR.setTargetPosition(distance);
            BL.setTargetPosition(-distance);
            FR.setTargetPosition(distance);
            FL.setTargetPosition(-distance);

            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
            {
                turnLeft(power);
            }
        }

        public void strafeLeft(double power)
        {
            BR.setPower(-power);
            BL.setPower(power);
            FR.setPower(power);
            FL.setPower(-power);
        }

        public void strafeLeftEncoders(double power, int distance)
        {
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BR.setTargetPosition(-distance);
            BL.setTargetPosition(distance);
            FR.setTargetPosition(distance);
            FL.setTargetPosition(-distance);

            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
            {
                strafeRight(power);
            }
        }

        public void strafeRight(double power)
        {
            BR.setPower(power);
            BL.setPower(-power);
            FR.setPower(-power);
            FL.setPower(power);
        }

        public void strafeRightEncoders(double power, int distance)
        {
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BR.setTargetPosition(distance);
            BL.setTargetPosition(-distance);
            FR.setTargetPosition(-distance);
            FL.setTargetPosition(distance);

            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
            {
                strafeLeft(power);
            }
        }

        public void park()
        {

           driveForwardEncoders(0.5, 1000);
           strafeRightEncoders(0.2, 3000);
        }







    }

