import SimpleOpenNI.*;
import oscP5.*;
import netP5.*;

import processing.sound.*;

SoundFile soundfile1;
SoundFile soundfile2;
SoundFile soundfile3;

SimpleOpenNI kinect;
NetAddress myRemoteLocation;
OscP5 oscP5Receiver;

int currentState; 
int prevState; 

//figure variables

float volatility; 
float flux; 
color poseColor;

//aura-ring variables

float avgX;
float avgY;
float size;
int formResolution = int(random(3, 10));
int stepSize = 5;
int count = 100;
float tempstrokeWeight = 1;
int look02 = 200;
float centerX, centerY;
float[] x = new float[formResolution];
float[] y = new float[formResolution];

void setup() {

  size(displayWidth, displayHeight, OPENGL);
  background(0);

  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableUser();

  myRemoteLocation = new NetAddress("127.0.0.1", 6448);  
  oscP5Receiver = new OscP5(this, 12000);

  soundfile1 = new SoundFile(this, "Pose_1.mp3");
  soundfile2 = new SoundFile(this, "Pose_2.mp3");
  soundfile3 = new SoundFile (this, "Pose_3.mp3");

  //aura-ring//
  strokeWeight(tempstrokeWeight);
  smooth();
  stroke(0, 50);
}

void getProjective(int userId, int jointID, PVector convertedJoint) {
  PVector joint = new PVector();
  float confidence = kinect.getJointPositionSkeleton(userId, jointID, joint);
  if (confidence < 0.5) {
    return;
  }
  kinect.convertRealWorldToProjective(joint, convertedJoint);
}

void draw() {
  scale(2);
  translate(150, 0); 
  fill(0, 15);
  rect(-155, 0, width, height);

  kinect.update();

  IntVector userList = new IntVector();
  kinect.getUsers(userList); 

  OscMessage myMessage = new OscMessage("/wek/inputs");

  if (userList.size() > 0) {

    for (int i = 0; i < userList.size(); i++) {

      int userId = userList.get(i); 

      ArrayList<PVector> joints = new ArrayList<PVector>();
      ArrayList<Integer> jointIds = new ArrayList<Integer>();

      jointIds.add(SimpleOpenNI.SKEL_LEFT_FOOT); //Id# = 13
      jointIds.add(SimpleOpenNI.SKEL_LEFT_KNEE); //Id# = 11
      jointIds.add(SimpleOpenNI.SKEL_LEFT_HIP); //Id# = 9
      jointIds.add(SimpleOpenNI.SKEL_TORSO); //Id# = 8
      jointIds.add(SimpleOpenNI.SKEL_LEFT_SHOULDER); //Id# = 2
      jointIds.add(SimpleOpenNI.SKEL_LEFT_ELBOW); //Id# = 4
      jointIds.add(SimpleOpenNI.SKEL_LEFT_HAND); //Id# = 6
      jointIds.add(SimpleOpenNI.SKEL_NECK); //Id# = 1
      jointIds.add(SimpleOpenNI.SKEL_HEAD); //Id# = 0
      jointIds.add(SimpleOpenNI.SKEL_NECK); //Id# = 1
      jointIds.add(SimpleOpenNI.SKEL_RIGHT_HAND); //Id# = 7
      jointIds.add(SimpleOpenNI.SKEL_RIGHT_ELBOW); //Id# = 5
      jointIds.add(SimpleOpenNI.SKEL_RIGHT_SHOULDER); //Id# = 3
      jointIds.add(SimpleOpenNI.SKEL_TORSO); //Id# = 8
      jointIds.add(SimpleOpenNI.SKEL_RIGHT_HIP); //Id# = 10
      jointIds.add(SimpleOpenNI.SKEL_RIGHT_KNEE); //Id# = 12
      jointIds.add(SimpleOpenNI.SKEL_RIGHT_FOOT); //Id# = 14

      for (int k = 0; k < jointIds.size(); k++) {
        int jid = jointIds.get(k);
        PVector newJoint = new PVector();
        getProjective(userId, jid, newJoint);
        joints.add(newJoint);
      }

      if (userList.size() == 2) {
        myMessage.add(joints.get(3).x);
        myMessage.add(joints.get(3).y);
        myMessage.add(joints.get(6).x - joints.get(3).x);
        myMessage.add(joints.get(6).y - joints.get(3).y);
        myMessage.add(joints.get(10).x - joints.get(3).x);
        myMessage.add(joints.get(10).y  - joints.get(3).y);
        myMessage.add(joints.get(8).x - joints.get(3).x);
        myMessage.add(joints.get(8).y - joints.get(3).y);
      }

      // State Changer 

      if (kinect.isTrackingSkeleton(userId)) {
        if (currentState == 1) {
          flux = 5;
          volatility = 0.3; 
          poseColor = lerpColor(color(76, 82, 240), color(7, 200, 249), 0.5+0.5*sin(frameCount*0.015));
        } else if (currentState == 2) {
          flux = 10;
          volatility = 0.5; 
          poseColor = (lerpColor(color(255, 7, 7), color(243, 73, 11), 0.5+0.5*sin(frameCount*0.015)));
        } else if (currentState == 3) {
          flux = 5;
          volatility = 0.5;
          poseColor = (lerpColor(color(156, 15, 178), color(172, 130, 203), 0.5+0.5*sin(frameCount*0.015)));

          avgX = ((joints.get(0).x + joints.get(1).x + joints.get(2).x + joints.get(3).x + joints.get(4).x + joints.get(5).x + joints.get(6).x + joints.get(7).x + joints.get(9).x + joints.get(10).x + joints.get(11).x + joints.get(12).x + joints.get(13).x + joints.get(14).x + joints.get(15).x + joints.get(16).x)/17);
          avgY = ((joints.get(0).y + joints.get(1).y + joints.get(2).y + joints.get(3).y + joints.get(4).y + joints.get(5).y + joints.get(6).y + joints.get(7).y + joints.get(9).y + joints.get(10).y + joints.get(11).y + joints.get(12).y + joints.get(13).y + joints.get(14).y + joints.get(15).y + joints.get(16).y)/17); 

          centerX = avgX; 
          centerY = avgY;

          float d = dist(joints.get(8).x, joints.get(8).y, joints.get(15).x, joints.get(15).y);
          float s = map(d, 400, height, 200, 800);
          float initRadius = random(350, s);
          float angle = radians(360/float(formResolution));

          for (int q=0; q<formResolution; q++) {
            x[q] = cos(angle*q) * initRadius;
            y[q] = sin(angle*q) * initRadius;
          }

          for (int k=0; k<count; k++) {
            stroke(0, (255/count*k), 120, 30);

            for (int q=0; q<formResolution; q++) {
              x[q] += random(-stepSize, stepSize);
              y[q] += random(-stepSize, stepSize);
              x[q] += random(cos(angle*q) * initRadius /look02);
              y[q] += random(sin(angle*q) * initRadius /look02);
            }

            noFill();
            beginShape();
            curveVertex(x[formResolution-1]+centerX, y[formResolution-1]+centerY);

            for (int q=0; q<formResolution; q++) {
              curveVertex(x[q]+centerX, y[q]+centerY);
            }
            curveVertex(x[0]+centerX, y[0]+centerY);
            curveVertex(x[1]+centerX, y[1]+centerY);
            endShape();
          }
        } else {
          flux = 5;
          volatility = 0.2; 
          poseColor = color(0, 0, 255);
        }

        float fatWidthOff = flux*sin(volatility*frameCount); 
        float thinWidthOff = flux*cos(volatility*frameCount);
        float fatHeightOff = flux*sin(volatility*frameCount); 
        float thinHeightOff = flux*cos(volatility*frameCount);

        for (int j = 0; j < joints.size(); j++) {
          fill(0); 
          ellipse(joints.get(j).x, joints.get(j).y, 5, 5);
        }

        pushStyle(); 
        fill(poseColor);
        createShape();
        beginShape(); 
        curveVertex(joints.get(1).x, joints.get(1).y); //Left Knee
        curveVertex(joints.get(0).x, joints.get(0).y); //Left Foot
        curveVertex(joints.get(0).x - fatWidthOff, joints.get(0).y); //Left Foot Offset
        curveVertex(joints.get(1).x - fatWidthOff, joints.get(1).y); //Left Knee Offset
        curveVertex(joints.get(2).x - fatWidthOff, joints.get(2).y); //Left Hip Offset
        curveVertex(joints.get(3).x - thinWidthOff, joints.get(3).y); //Torso Offset
        curveVertex(joints.get(4).x, joints.get(4).y + thinHeightOff); //Left Shoulder offset
        curveVertex(joints.get(5).x - thinWidthOff, joints.get(5).y); //Left Elbow
        curveVertex(joints.get(6).x - thinWidthOff, joints.get(6).y); //Left Hand
        curveVertex(joints.get(6).x - thinWidthOff, joints.get(6).y - fatHeightOff); //Left Hand Offset
        curveVertex(joints.get(5).x - thinWidthOff, joints.get(5).y - fatHeightOff); // Left Elbow Offset
        curveVertex(joints.get(4).x, joints.get(4).y - thinHeightOff); //Left Shoulder
        curveVertex(joints.get(7).x - thinWidthOff, joints.get(7).y); //Neck Offset  
        curveVertex(joints.get(8).x - fatWidthOff, joints.get(8).y); //Head Offset
        curveVertex(joints.get(8).x, joints.get(8).y - fatHeightOff); //Head Offset
        curveVertex(joints.get(8).x + fatWidthOff, joints.get(8).y); //Head Offset
        curveVertex(joints.get(9).x + thinWidthOff, joints.get(9).y); //Neck Offset
        curveVertex(joints.get(12).x, joints.get(12).y - thinHeightOff); //Right Shoulder
        curveVertex(joints.get(11).x + thinWidthOff, joints.get(11).y - fatHeightOff); //Right Elbow Offset
        curveVertex(joints.get(10).x + thinWidthOff, joints.get(10).y - fatHeightOff); //Right Hand Offset
        curveVertex(joints.get(10).x + thinWidthOff, joints.get(10).y); //Right Hand
        curveVertex(joints.get(11).x + thinWidthOff, joints.get(11).y); //Right Elbow
        curveVertex(joints.get(12).x, joints.get(12).y + thinHeightOff); //Right Shoulder Offset
        curveVertex(joints.get(13).x + thinWidthOff, joints.get(13).y); //Torso Offset
        curveVertex(joints.get(14).x + fatWidthOff, joints.get(14).y); //Right Hip Offset
        curveVertex(joints.get(15).x + fatWidthOff, joints.get(15).y); //Right Knee Offset
        curveVertex(joints.get(16).x + fatWidthOff, joints.get(16).y); //Right Foot Offset
        curveVertex(joints.get(16).x, joints.get(16).y); //Right Foot
        curveVertex(joints.get(15).x, joints.get(15).y); //Right Knee
        endShape(CLOSE); 
        popStyle();
      }
    }
  }

  if (userList.size() == 2) {
    oscP5Receiver.send(myMessage, myRemoteLocation);
  }

  pushStyle();
  textSize(64);
  fill(225, 0, 0);
  text(currentState, width/2, height/2);
  popStyle();

  if (currentState != prevState) {
    soundfile1.stop(); 
    soundfile2.stop(); 
    soundfile3.stop();
    if (currentState == 1) {
      soundfile1.loop();
    } else if (currentState == 2) {
      soundfile2.loop();
    } else if (currentState == 3) {
      soundfile3.loop();
    }
  }
}

void oscEvent(OscMessage message) {
  if (message.checkAddrPattern("/wek/outputs") == true) {
    currentState = (int) message.get(0).floatValue();
  }
}

void onNewUser(SimpleOpenNI kinect, int userID) {
  println("Start skeleton tracking");
  kinect.startTrackingSkeleton(userID);
}
