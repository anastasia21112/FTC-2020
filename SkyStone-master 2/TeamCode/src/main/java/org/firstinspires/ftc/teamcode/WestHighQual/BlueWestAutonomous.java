/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.WestHighQual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="BlueWestAutonomous", group ="Linear Opmode")
@Disabled

public class BlueWestAutonomous extends LinearOpMode {

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;




    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY = "AdrbCxn/////AAABmbek7BzvFEMZmkp+aKPHeZlW4J5TCb8M0FVzUQzg8S2yk9DcyFljlN4Ek0JNm9CxCbT5GX88EC2/sZWfl3Sq4kpeLtOoY24cSmDKSrzMGHombdGWM0CQCutT2F5XhGOswfCGOjmB+BZWvv3vdX3jb9WFhtQbayUtsd8vcfMty0Ww6/cOUpZNiRfHvLH8GQqJ0gTaBvjyCU8If2iMdmltCd1LF7jROQGbwgKWvJAoooFqrWIihkF/iMHlLiUXbIQVupizQgE0ZcRjiS4TF3yx2rMa27kvrvBABZHMpwecZerZlBP8JMrtQMlSAl5TWY+8RTUlTSSODCEfrujAehCloeg6m885KfwtMmR8bHWOfLwH";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    //Mechanical Devices Used
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private Servo IL;
    private Servo IR;
    private DcMotor LiR;
    private DcMotor LiL;
    private Servo drag;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override public void runOpMode() {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        FL = hardwareMap.get(DcMotor.class, "Front Left");
        FR = hardwareMap.get(DcMotor.class, "Front Right");
        BR = hardwareMap.get(DcMotor.class, "Back Right");
        BL = hardwareMap.get(DcMotor.class, "Back Left");
        IL = hardwareMap.get(Servo.class, "Intake Left");
        IR = hardwareMap.get(Servo.class, "Intake Right");
        LiR = hardwareMap.get(DcMotor.class, "Linear Slide Right");
        LiL = hardwareMap.get(DcMotor.class, "Linear Slide Left");
        drag = hardwareMap.get(Servo.class, "Drag");

        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        waitForStart();
        //prepare to find target

        driveBackwardEncoders(1.0, 300);
        sleep(5000);

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        String positionSkystone = "";
        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    if(trackable.getName().equals("Stone Target"))
                    {
                        telemetry.addLine("Stone Target Is Visible");
                    }

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }


            }

            // Provide feedback as to where the robot is located (if we know).

            if (targetVisible) {

                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                telemetry.update();
                double yPosition = translation.get(1)/mmPerInch;




                if(yPosition < 7.0 && yPosition > 0.1)
                {
                    positionSkystone = "center";
                    telemetry.addData("Center", positionSkystone, yPosition);
                    telemetry.update();
                    driveBackwardEncoders(1.0, 345);
                    strafeRightEncoders(1.0, 150);
                    grabBlock();
                    dragBlock();
                    park();

                }
                else if(yPosition < 0.1)
                {
                    positionSkystone = "left";
                    telemetry.addData("left", positionSkystone);
                    telemetry.update();
                    driveBackwardEncoders(1.0, 345);
                    strafeLeftEncoders(1.0, 50);
                    grabBlock();
                    dragBlock();
                    park();

                }
                else
                {

                        positionSkystone = "right";
                        telemetry.addData("right", positionSkystone);
                        telemetry.update();
                        driveBackwardEncoders(1.0, 300);
                        strafeLeftEncoders(1.0, 300);
                        grabBlock();
                        dragBlock();
                }
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            }
            else {
                telemetry.addData("Visible Target", "none");
                positionSkystone = "right";



                positionSkystone = "right";
                telemetry.addData("right", positionSkystone);
                telemetry.update();
                driveBackwardEncoders(1.0, 300);
                strafeRightEncoders(1.0, 300);
                grabBlock();
                dragBlock();




            }

            telemetry.addData("Skystone Position: ", positionSkystone);

            telemetry.update();


        }


        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();

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

    public void strafeLeftEncoders(double power, int distance )
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
            strafeRight(power);
        }
    }

    public void grabBlock()
    {
        drag.setPosition(1.0);
        sleep(2800);
        drag.setPosition(0.5);
        sleep(300);
        driveForwardEncoders(1.0, 300);
    }

    public void dragBlock()
    {
        for(int i = 0; i < 7; i++)
        {
            strafeRightEncoders(1.0, 300);
            turnRightEncoders(1.0, 10);
            sleep(100);

        }
        drag.setPosition(0.0);
        sleep(100);

    }

    public void park()
    {
        for(int i = 0; i < 2; i++) {
            strafeRightEncoders(1.0, 350);
            turnRightEncoders(1.0, 10);
        }
    }


    @Autonomous(name = "Park Blue", group="Linear OpMode")
    public static class Park extends LinearOpMode
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
                    park();

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

                public void park()
                {
                    driveForwardEncoders(1.0, 150);
                    turnLeftEncoders(1.0, 100);
                    Drag.setPosition(1.0);
                    sleep(1000);
                    Drag.setPosition(0.5);
                }







            }
}
