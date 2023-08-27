package org.firstinspires.ftc.teamcode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robots.ToBeNamed;

@TeleOp(name="CameraTest", group = "Test")
public class CameraTest extends LinearOpMode {
    ToBeNamed control;

    /*Vuforia*/
    WebcamName webcamName;
    private VuforiaLocalizer vuforia; //feeding the image2D only for tensorflow to use
    private static final String VUFORIA_KEY =
            "AY6BsCf/////AAABmZ6ima003kgksPYl8C+B8VZ1LH2yueEfFxy4p14SPPRdHkGuEtSIewtOX5QIPU6XkSPde" +
                    "k3tAIssoQkq1jF8hmsZINtLhHYNPojqBvsizbouwQTwhRm+Xej0KFhPo5yvOTJZRuJ5faItG0UGRTJ5u3wpfaWohaEyDwFgl" +
                    "DsYeTIhp0zXk0cVhjctUpYPnd5advw0jRBfEoa5GV+rHi/kxPzvzyvPrVevqzLyRPRDBMneVn6MnD9/Nyvb5QUh9ZGnRBZxT" +
                    "5ilYq1yWiP9R98pZdYnGwYStzkz+hZDHWluCwwduRF4blVS2W6jgC0RZfqMWT+7rG58RdpjjhzH7CYkcIW2R256kPTP9b85O" +
                    "prB1eap";
    /*Object Detection (TensorFlow)*/
    private TFObjectDetector tfod; //tensorflow object detection
    private static final String TFOD_MODEL_ASSET = "";
    private static final String[] LABELS = {};

    public void runOpMode(){
        /*init*/
        initVuforia();
        initTF_ObjectDetector();

        waitForStart();
    }

    private void initVuforia(){
        //https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/configuring_external_webcam/configuring-external-webcam.html
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = true;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTF_ObjectDetector(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.useObjectTracker = true;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
