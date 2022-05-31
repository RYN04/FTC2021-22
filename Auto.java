package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import java.util.List;

@Autonomous(name = "Auto2", group = "Concept")
public class Auto extends LinearOpMode {
    //Variable Declarations & Initialization
    private static final String TFOD_MODEL_ASSET = "model_20220127_141950.tflite";
    private static final String[] LABELS = {"TSE"};
    private static final String VUFORIA_KEY =
            "AR3FoLn/////AAABmbTKu26tsEoKlcvQBB9zGXciWoHkYZeO9i06nfa4b6CJUla439VwG17+eH6q+zMu+u3zbngUS36mv0RxW2bGx4fbTxgatgNRXEwq9vSfHClyZv1P8+gVLuKrLim339ymYDkG/KxzYBS+BXCH6x7WAMJtJwWXjbh5ld+VXArigoaFNAW66sAh2rp+kgauU+i1oMe+HavxukrUj4bjj3x33jvhiP4EILgS4WXRLK1X8+aa8j1iWOI7yy8c81XcM8uzF61IKhBfxohGS2JML/96iOdIdUtboOtPOwI7FEQS1TE7/YXcsehGsu98CS8CVarmRFla5a1C2dfQh8NAIjTJ0ogpft2p839BoVxGqF16gkm2";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private DcMotorEx arm_motor1;
    private DcMotorEx arm_motor2;
    private DcMotorEx ddMotor;

    private Servo left_servo;
    private Servo right_servo;

    private float pos;
    private int con;

    @Override
    public void runOpMode() {
        //Configuring
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ddMotor = hardwareMap.get(DcMotorEx.class, "duckDeliveryMotor");
        arm_motor1 = hardwareMap.get(DcMotorEx.class, "armMotor1");
        arm_motor2 = hardwareMap.get(DcMotorEx.class, "armMotor2");
        left_servo = hardwareMap.servo.get("LeftServo");
        right_servo = hardwareMap.servo.get("RightServo");

        //initialize Vuforia & Tfod
        initVuforia();
        initTfod();

        //turning on the webcam with zoom setup
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }//end if

        waitForStart();

        //initial position of the robot
        drive.setPoseEstimate(new Pose2d(-35, 64.5, Math.toRadians(-90)));
        
        ///////////////////////////////////////////////////////////////////////////////////////
        //ALL the Trajectories should be written HERE
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-35, 64.5, Math.toRadians(-90)))
                .forward(3)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(85))))
                .back(23,
                        //slowing down the speed for this trajectory only
                        SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .addDisplacementMarker(() -> {
                    duckSpin(8500, 1);
                })
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2. end())
                .forward(5)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(0, 27.5, 0))
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .forward(46.5)
                .addDisplacementMarker(() -> {
                    if (con == 1)
                    {
                        moveArm(-245, 0.5);
                    }
                    else if (con == 2)
                    {
                        moveArm(-570, 0.5);
                    }
                    else
                    {
                        moveArm(-1000, 0.5);
                    }
                })
                .waitSeconds(0.2)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end().plus(new Pose2d(0, 0, Math.toRadians(-3))))
                .back(48,
                        //slowing down the speed for this trajectory only
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0.1, () -> {
                    closeServo();
                })
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(0, 5, Math.toRadians(-93)))
                .addTemporalMarker(0.2, () -> {
                    moveArm(1000, 0.5);
                })
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(0, 33, Math.toRadians(-3)))
                .addTemporalMarker(0.1, () -> {
                    openServo();
                })
                .addTemporalMarker(0.15, () -> {
                    moveArm(250, 0.5);
                })
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(46)
                .addDisplacementMarker(() -> {
                        moveArm(-1000, 0.5);
                })
                .build();
        ///////////////////////////////////////////////////////////////////////////////////////

        //Code below will be executed after pressing play.
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    //set up the List of objects detected.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    /*
                    If you want to check the position of the object,
                    Check with CameraStreamCheck.java in this repository.
                    Before Running, Always make SURE that the object is in the range you chose.
                    */
                    if (updatedRecognitions != null) {
                        for (Recognition recognition : updatedRecognitions) {
                            //assign value to the variable pos.
                            pos = recognition.getLeft();
                        }//end for loop

                        closeServo();

                        if (isStopRequested()) return;

                        //ALL .followTrajectory starts from HERE
                        drive.followTrajectory(traj1);
                        turn(85, drive);
                        drive.followTrajectorySequence(traj2);
                        drive.followTrajectory(traj3);

                        //when there is no object detected
                        if (updatedRecognitions.size() == 0) {
                            level3(drive, traj3);
                            //when there is one object detected
                        } else if (updatedRecognitions.size() == 1) {
                            //when the position of the object is in-between this range
                            if (-20 < pos && pos < 100) {
                                level1(drive, traj3);
                            } else if (200 < pos) {
                                level2(drive, traj3);
                            }
                        }
                        //when there is two or more object detected,
                        //made it to run to level3 as default.
                        else
                        {
                            level3(drive, traj3);
                        }

                        openServo();

                        //set new Position estimate for convenience
                        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));

                        drive.followTrajectory(traj4);
                        drive.followTrajectorySequence(traj5);
                        drive.followTrajectory(traj6);
                        drive.followTrajectory(traj7);
                        drive.followTrajectory(traj8);
                        drive.followTrajectory(traj9);
                        
                        //It is IMPORTANT to put long sleep after all the movement is completed
                        sleep(10000);
                    }//end if (updatedRecognitions != null)
                }//end if (tfod != null)
            }//end while
        }//end if
    }//end runOpMode()

    //ALL the methods belong below
    private void level1(SampleMecanumDrive drive, Trajectory traj3) {
        con = 1;
        Trajectory lev1 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-14.5, 57.5, Math.toRadians(-95)))
                .addTemporalMarker(0.01, () -> {
                    moveArm(570, 0.5);
                })
                .addTemporalMarker(0.8, () -> {
                    moveArm(-325, 0.5);
                })
                .build();
        drive.followTrajectory(lev1);
    }//end level1()

    private void level2(SampleMecanumDrive drive, Trajectory traj3) {
        con = 2;
        Trajectory lev2 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-14.5, 57.5, Math.toRadians(-95)))
                .addTemporalMarker(0.4, () -> {
                    moveArm(570, 0.5);
                })
                .build();
        drive.followTrajectory(lev2);
    }//end level2()

    private void level3(SampleMecanumDrive drive, Trajectory traj3) {
        con = 3;
        Trajectory lev3 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-14.5, 53, Math.toRadians(-95)))
                .addTemporalMarker(0.4, () -> {
                    moveArm(1000, 0.5);
                })
                .build();
        drive.followTrajectory(lev3);
    }//end level3()

    private void duckSpin(int position, double power) {
        ddMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ddMotor.setTargetPosition(position);
        ddMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ddMotor.setPower(power);
    }//end duckSpin()

    private void moveArm(int position, double power) {
        arm_motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor1.setTargetPosition(position);
        arm_motor2.setTargetPosition(position);
        arm_motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_motor1.setPower(power);
        arm_motor2.setPower(power);
    }//end moveArm()

    private void openServo() {
        left_servo.setPosition(0.17);
        right_servo.setPosition(0.31);
    }//end openServo()

    private void closeServo() {
        left_servo.setPosition(0.4);
        right_servo.setPosition(0.08);
    }//end closeServo()

    private void turn(double angle, SampleMecanumDrive drive) {
        drive.turn(Math.toRadians(angle));
    }//end turn()

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }//end initVuforia()

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }//end initTfod()
}//end class
