import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import SimpleOpenNI.*; 
import ipcapture.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Kinect extends PApplet {

//import processing.net.*;



ipcapture.IPCapture cam;

SimpleOpenNI kinect;
//Client client;
int[] userMapArr;
float tolerence = 0.6f;
int i = 0;

public void setup(){
	frameRate(15);
	size(640,480);

	//client to send to node server
	//client = new Client(this,"172.30.40.34",3000);

	//kinect
	kinect = new SimpleOpenNI(this);
	kinect.enableDepth();
	kinect.enableUser();
	kinect.setMirror(true);

}

public void draw(){
	background(255);

	//kinect image
	kinect.update();
	image(kinect.userImage(), 0, 0);

	int[] userList = kinect.getUsers();
	for (int i = 0; i < userList.length; ++i) {
		if(kinect.isTrackingSkeleton(userList[i]))
			drawSkelet(userList[i]);
	}

	//camera
	if (cam.isAvailable()) {
    	cam.read();
    	image(cam,0,0);
  	}
}

public void drawSkelet(int userId){
	PVector rightHandPos = new PVector();
	float confidenceRight = kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_HAND , rightHandPos);
	PVector projRightHandPos = new PVector();
	if(confidenceRight > tolerence){
		fill(0,255,0);
		kinect.convertRealWorldToProjective(rightHandPos,projRightHandPos);
		ellipse(projRightHandPos.x, projRightHandPos.y, 20, 20);
	}

	PVector leftHandPos = new PVector();
	float confidenceLeft = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,leftHandPos);
	PVector projLeftHandPos = new PVector();
	if(confidenceLeft > tolerence){
		fill(255,0,0);
		kinect.convertRealWorldToProjective(leftHandPos,projLeftHandPos);
		ellipse(projLeftHandPos.x, projLeftHandPos.y, 20, 20);
	}

	line(projLeftHandPos.x, projLeftHandPos.y, projRightHandPos.x, projRightHandPos.y);

	float midX=projLeftHandPos.x+((projRightHandPos.x-projLeftHandPos.x)/2.0f);
	float midY=projLeftHandPos.y+((projRightHandPos.y-projLeftHandPos.y)/2.0f);

	ellipse(midX, midY, 20, 20);

	line(midX - 640,midY,midX+640,midY);
	ellipse(640, midY, 20, 20);

	float a = calculateDistance(640, midY, projLeftHandPos.x, projLeftHandPos.y);
	float b = calculateDistance(midX, midY, projLeftHandPos.x, projLeftHandPos.y);
	float c = calculateDistance(midX, midY, 640, midY);

	float iA = a * a;
	float iB = b * b;
	float iC = c * c;

	float teller = iB + iC - iA;
	float noemer = 2 * b * c;

	float cosA = teller / noemer;
	float degree = degrees(acos(cosA));

	if (projLeftHandPos.y > midY) {
		degree = -degree;
	}

	if(i%10 == 0){
		i = 0;
		//client.write(degree+"/");
	}
	i++;
}

public void onNewUser(SimpleOpenNI kinect, int userId){
	kinect.startTrackingSkeleton(userId);
}

public float calculateDistance (float x1, float y1, float x2, float y2) {
	float notC = ((x1-x2) * (x1-x2)) + ((y1-y2) * (y1-y2));
	float c = sqrt(notC);

	return c;
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Kinect" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
