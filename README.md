This repository contains codes that are primarily written based on Road runner library.

## The following writings will explain about 

#### (1) PID tuning using RoadRunner, 
#### (2) setting up Object detection, 
#### (3) and coding using Roadrunner trajectories. 

## 1. PID tuning using RoadRunner

[**Road Runner website**](https://learnroadrunner.com)

Road runner is the tool that allows coordinate-based coding using complex path following and generation 
while maintaining control of velocity, heading, and acceleration that is obtained from tuning process.

Try setting up AndroidStudio yourself, and text 
**@Ryn** or **@Alexandra** in Discord if you are stuck.
You can also reach out to **FTC discord programming help channel**.

[**Installing RoadRuner in Android Studio**](https://learnroadrunner.com/installing.html#method-1-downloading-the-quickstart)

One of the common mistake with Android Studio is misunderstanding how it works. The process is like this;
```
1. Import the FTC project in Android Studio
2. Connect your device with the robot
3. press play button in Android Studio to build and upload the code.
```
So everytime you make changes in your code, you have to upload the code to the robot. 
This takes some time compared to **onBotJava** but it is still safer to code with A.S. as the code in onBotjava sometimes disappear.

Our tuning process was like this:
```
1. DriveConstants
2. DriveFeedforwardTuner
3. StraightTest & StrafeTest
4. TrackWidthTuner
5. TurnTest
6. FolowerPIDTuner
7. SplineTest
```

Before tuning, if [**192.168.43.1:8080/dash**](https://192.168.43.1:8080/dash) link does not work, this means that you have not successfully uploaded the code from Android Studio 
or your device is not connected to the robot's wifi.
Try setting up Android Studio again.

If you have done setting up A.S. and dashboard, watch this [**helpful video**](https://www.youtube.com/watch?v=7wjaX2KXrrM) that tunes the robot from beginning.

**For Step 2**, it is written that if you are using encoders, you have to tune the robot using DriveVelocityPIDTuner,

however I recommend using **DriveFeedforwardTuner** as it is eaiser to tune.

It is okay to use **DriveFeedforwardTuner** even when using encoders.
When tuning with **DriveFeedforwardTuner**, 
it is important to make **enough long plateau** in graph. 
If you are not in the condition to have enough plateau, you'll have to reduce your max velocity.

**For Step 3** StrafeTest, our team failed to tune the robot to strafe straight right/left. We was able to match the distance but the direction was quite messed up. I believe this was either the problem of the weight distribution or wheel attachment of our robot. However, the goal of this process is to match the distance not the heading so if you got the distance right, then you'll be fine. (+ Heading can manually adjusted while coding the autnomous, so dw)

**For Step 4** TrackWidthTuner, MaxAngularVelocityTuner was broken (I believe every team have experienced this), so we had to manually estimate our max angular velocity and get into tuning.

**For Step 5** TurnTest, your goal is to make an error range of 1~3 degree. Once you got into that range, you'll be fine.

## 2. ObjectDetection

In this repository you'll find two following codes **1. Auto.java, 2. CameraStreamCheck**

Auto.java is the acutal code for autonomous that contains functions of Object detection/Road runner trajectories.
CameraStreamCheck is the one I modified the [**sample code**](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptTensorFlowObjectDetectionWebcam.java) that is provided in FTC library.

you MUST read over this code and try to understand each part of it.

Refer to CameraStreamCheck when you are trying to set up the camera and object detection.
One of our biggest challenge for this competition was creating a TensorFlow file which models our custom element.
Through that .tflite model, the camera detects where the object is and is able to return the position of our object.
If you also have to detect the element, and you are using custom object, 

refer to this [**document on FTC machine learning tool**](https://storage.googleapis.com/ftc-ml-firstinspires-prod/docs/ftc-ml_manual_2021.pdf)

One thing you have to know to use this machine learning tool is that you have to log in with **coach's account**.

Ask Ms.Annetta's account and log in.

Common error you would see is that the model you made does not properly models the object. This error is mostly solved by increasing the steps for training.

After done creating the file, look at **pg.23** of that document to use the file in Android Studio.

## 3. Coding Autonomous using RoadRunner Trajectories

If you have done the tuning in part 1, now it's time to code using trajectories!

Try to work with Alexandra as she has experience with trajectories, and she codes well

I know it is hard to be organized while coding, but I highly recommend you to write the code in organized and readible way so that team for next year can look at the code and understand better. It would be nice to follow in style I wrote.

Before getting into coding, let's have a look what Trajectory does.

#### 1. Trajectories

[**Road Runner Trajectories**](https://learnroadrunner.com/trajectories.html#trajectories-vs-paths)

**Concept of Trajectory**
```
SampleMecanumDrive drive = new SampleMecanumDrive drive(hardwareMap);
Trajectory traj = drive.trajectoryBuilder(new Pose2d())
    .forward(3)
    .build()
```
The code above projects the trajectory movement of going forward for the value of 3.
The first line of code is simply creating the SampleMecanumDrive object which you have modified during the tuning process.
From second line, you are creating the trajectory object that projects the movement. 

To make the robot to Actually move, add this line below.
```
drive.followTrajectory(traj);
```
This is the actual function that makes the robot to move in that trajectory.

Not only going forward and backward, RoadRunner trajectories offer complex movements such as splining and turning while moving.
You can find those movements in [TrajectoryBuilder Function List](https://learnroadrunner.com/trajectorybuilder-functions.html#splinetosplineheading-endpose-pose2d-endtangent-double)

Also, if your robot's initial position is not a center of the field, you'll have to set the inial pose estimate.
```
drive.setPoseEstimate(startPose)
```

#### 2. Consecutive Trajectories

Let's say you have two consecutive movements such as going forward and then splining to destination.
In this case, if you use .end(), you can easily 'link' those two trajectories

(It is Possible to put two movement methods in one trajectory but I prefered to seperate them)

```
Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();
Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .lineToSplineHeading(new Pose2d(0, 27.5, 0))
                .build();
```

traj2 will start from the location where traj ended.

Also, note that turn(angle) function is not part of the RoadRunner Trajectory.
This means that you have to manually add the angle in the Pose2d()

```
Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-35, 64.5, Math.toRadians(-90)))
                .forward(3)
                .build();
Trajectory traj2 = drive.trajectorySequenceBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .back(3))
                .build();
drive.followTrajectory(traj1);
drive.turn(Math.toRadians(90));
dirve.followTrajectory(traj2);
```
From traj1.end() add 90 angle turn by putting .plus(newPose2d(0, 0, Math.toRadians(90)));

#### 3. Using Markers to allow other actions (Arm up and down, servo, shooting, etc..)

If you want to move arm and down, or do other stuff, 

you have to add [**Markers**](https://learnroadrunner.com/markers.html#types-of-markers) in the trajectory. 
If your desired action does not involve encoders, it is okay to put that line of action outside of the trajectory, but if encoders involved, it is neccessary to use markers.

There are two types of Markers in general, 

1. [addDisplacementMarker()](https://learnroadrunner.com/markers.html#displacement-markers-basics)

2. [.addTemporalMarker()](https://learnroadrunner.com/markers.html#types-of-markers)

