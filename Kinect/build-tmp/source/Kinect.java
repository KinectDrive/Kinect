import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import SimpleOpenNI.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Kinect extends PApplet {


//Client client;

SimpleOpenNI kinect;
int[] userMapArr;
float tolerence = 0.6f;
int i = 0;

public void setup(){
	frameRate(15);
	size(640,480);

	//client = new Client(this,"172.30.40.35",3000);
	kinect = new SimpleOpenNI(this);
	kinect.enableDepth();
	kinect.enableUser();
	kinect.setMirror(true);
}

public void draw(){
	background(255);
	kinect.update();
	image(kinect.userImage(), 0, 0);

	int[] userList = kinect.getUsers();
	for (int i = 0; i < userList.length; ++i) {
		if(kinect.isTrackingSkeleton(userList[i]))
			drawSkelet(userList[i]);
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

	//midX=startX+((endX-startX)/2.0);
	//midY=startY+((endY-startY)/2.0);
	float midX=projLeftHandPos.x+((projRightHandPos.x-projLeftHandPos.x)/2.0f);
	float midY=projLeftHandPos.y+((projRightHandPos.y-projLeftHandPos.y)/2.0f);

	ellipse(midX, midY, 20, 20);

	line(midX - 640,midY,midX+640,midY);
	ellipse(640, midY, 20, 20);

	//client.write(projLeftHandPos.x+"/");

	// cos^-1(-a^2 + b^2+c^2 / 2b);
	float a = dist(leftHandPos.x, leftHandPos.y, 640, midY);
	float b = dist(midX, midY, leftHandPos.x,leftHandPos.y);
	float c = dist(midX, midY, 640,midY);

	//println("a: "+a);
	//println("b: "+b);
	//println("c: "+c);

	float iA = pow(a, 2);
	float iB = pow(b, 2);
	float iC = pow(c, 2);

	float cos = (-iA + iB + iC) / (2 * b * c);

	float angle = acos(cos);

	float graden = (angle * 180)/(PI);

	if(i%100 == 0){
		println("Rad: "+ cos);
		i = 0;
	}
	i++;
	//println("Rad: "+ cos);
}

public void onNewUser(SimpleOpenNI kinect, int userId){
	kinect.startTrackingSkeleton(userId);
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
