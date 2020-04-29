/* Team 16748
 * Lucas Erickson
 */

package org.firstinspires.ftc.teamcode.StateFinalCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class RedAllianceSingleSkystoneAuto extends LinearOpMode {

    private OpenCvCamera phoneCam;

    private ImprovedSkystoneDetector skystoneDetector;

    private DcMotor FL;                              //declare OpMode mechanical members and their names used in the program
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor IR;
    private DcMotor IL;

    @Override
    public void runOpMode()
    {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        IL = hardwareMap.get(DcMotor.class, "IL");
        IR = hardwareMap.get(DcMotor.class, "IR");

        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        skystoneDetector = new ImprovedSkystoneDetector(true);


        phoneCam.setPipeline(skystoneDetector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("Stone four:", skystoneDetector.stoneFourisSkystone());
            telemetry.addData("Stone five:", skystoneDetector.stoneFiveisSkystone());
            telemetry.addData("Stone three:", skystoneDetector.stoneSixisSkystone());

            if(skystoneDetector.stoneFourisSkystone())
                telemetry.addData("Configuration:", "1 & 4");
            if(skystoneDetector.stoneFiveisSkystone())
                telemetry.addData("Configuration:", "2 & 5");
            if(skystoneDetector.stoneSixisSkystone())
                telemetry.addData("Configuration:", "3 & 6");

            telemetry.update();

            if(skystoneDetector.stoneFourisSkystone())
            {
                strafeLeftEncoders(0.2, 500);
                IR.setPower(1.0);
                IL.setPower(-1.0);
                driveForwardEncoders(1.0, 3500);
                driveForwardEncoders(0.5, 500);

                sleep(2000);
                //driveForwardEncoders(0.8, 1000);
                driveBackwardEncoders(-1.0, 2000);
                turnRightEncoders(1.0, 1300);
                driveForwardEncoders(1.0, 4800);
                IR.setPower(-1.0);
                IL.setPower(1.0);
                sleep(1000);
                driveBackwardEncoders(-1.0, 2000);


                break;
            }
            else if(skystoneDetector.stoneFiveisSkystone())
            {
                IR.setPower(1.0);
                IL.setPower(-1.0);
                driveForwardEncoders(1.0, 3500);
                driveForwardEncoders(0.5, 500);

                sleep(2000);
                //driveForwardEncoders(0.8, 1000);
                driveBackwardEncoders(-1.0, 2000);
                turnRightEncoders(1.0, 1300);
                driveForwardEncoders(1.0, 4800);
                IR.setPower(-1.0);
                IL.setPower(1.0);
                sleep(1000);
                driveBackwardEncoders(-1.0, 2000);


                break;
            }
            else
            {
                strafeRightEncoders(0.2, 700);
                IR.setPower(1.0);
                IL.setPower(-1.0);
                driveForwardEncoders(1.0, 3500);
                driveForwardEncoders(0.5, 500);

                sleep(2000);
                //driveForwardEncoders(0.8, 1000);
                driveBackwardEncoders(-1.0, 2000);
                turnRightEncoders(1.0, 1300);
                driveForwardEncoders(0.8, 4800);
                IR.setPower(-1.0);
                IL.setPower(1.0);
                sleep(1000);
                driveBackwardEncoders(-1.0, 2000);

                break;
            }
        }
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

        BR.setTargetPosition(-distance);
        BL.setTargetPosition(-distance);
        FR.setTargetPosition(-distance);
        FL.setTargetPosition(-distance);

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
}