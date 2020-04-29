package org.firstinspires.ftc.teamcode.WestHighQual.AutonomousSetUp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Encoders", group="Linear OpMode")
public class Encoders extends LinearOpMode
        {
            ElapsedTime runtime = new ElapsedTime();
            public DcMotor FR;
            public DcMotor FL;
            public DcMotor BR;
            public DcMotor BL;
            private Servo Drag;
            @Override
            public void runOpMode()
            {
                //name motors
                FR = hardwareMap.get(DcMotor.class, "Front Right");
                FL = hardwareMap.get(DcMotor.class, "Front Left");
                BR = hardwareMap.get(DcMotor.class, "Back Right");
                BL = hardwareMap.get(DcMotor.class, "Back Left");
                Drag = hardwareMap.get(Servo.class, "Drag");

                //configure motors
                BR.setDirection(DcMotor.Direction.FORWARD);
                BL.setDirection(DcMotor.Direction.REVERSE);
                FL.setDirection(DcMotor.Direction.REVERSE);
                FR.setDirection(DcMotor.Direction.FORWARD);

                //wait for program to start
                waitForStart();
                runtime.reset();

                //autonomous time
                /*
                driveBackwardEncoders(1, 300);
                driveBackwardEncoders(1.0, 450);
                sleep(500);
                driveBackwardEncoders(0.5, 100);
                driveForwardEncoders(1.0, 500);
                strafeLeftEncoders(0.7, 1500);
                turnLeftEncoders(0.5, 100);
                strafeLeftEncoders(0.7, 900);
                driveBackwardEncoders(1.0, 200);
                Drag.setPosition(1.0);
                sleep(1000);
                driveForwardEncoders(1.0, 500);
                turnLeftEncoders(1.0, 300);
                driveBackwardEncoders(1.0, 300);
                */




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

                BR.setTargetPosition(-distance);
                BL.setTargetPosition(-distance);
                FR.setTargetPosition(-distance);
                FL.setTargetPosition(-distance);

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
                BR.setPower(power);
                BL.setPower(-power);
                FR.setPower(-power);
                FL.setPower(power);
            }

            public void strafeLeftEncoders(double power, int distance)
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
                    strafeRight(power);
                }
            }

            public void strafeRight(double power)
            {
                BR.setPower(-power);
                BL.setPower(power);
                FR.setPower(power);
                FL.setPower(-power);
            }

            public void strafeRightEncoders(double power, int distance)
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
                    strafeLeft(power);
                }
            }






        }