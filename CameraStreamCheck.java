package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;

@Autonomous(name = "CameraStreamCheck", group = "Concept")
public class CameraStreamCheck extends LinearOpMode {
    /* For detailed explanation and set-up for this code, have a look on this sample code,
    https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptTensorFlowObjectDetectionWebcam.java
    */
    private static final String TFOD_MODEL_ASSET = "model_20220127_141950.tflite";
    private static final String[] LABELS = {"TSE"};
    private static final String VUFORIA_KEY =
            "AR3FoLn/////AAABmbTKu26tsEoKlcvQBB9zGXciWoHkYZeO9i06nfa4b6CJUla439VwG17+eH6q+zMu+u3zbngUS36mv0RxW2bGx4fbTxgatgNRXEwq9vSfHClyZv1P8+gVLuKrLim339ymYDkG/KxzYBS+BXCH6x7WAMJtJwWXjbh5ld+VXArigoaFNAW66sAh2rp+kgauU+i1oMe+HavxukrUj4bjj3x33jvhiP4EILgS4WXRLK1X8+aa8j1iWOI7yy8c81XcM8uzF61IKhBfxohGS2JML/96iOdIdUtboOtPOwI7FEQS1TE7/YXcsehGsu98CS8CVarmRFla5a1C2dfQh8NAIjTJ0ogpft2p839BoVxGqF16gkm2";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }//end if
        
        waitForStart();
        
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        //code for moving the robot belongs below
                        telemetry.update();
                    }//end if
                }//end if (tfod != null)
            }//end while
        }//end if
    }//end runOpMode()

    private void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }//end initVuforia()

    private void initTfod()
    {
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
