import processing.net.*;
import SimpleOpenNI.*;
import ipcapture.*;

ipcapture.IPCapture cam;

SimpleOpenNI kinect;
Client client;
int[] userMapArr;
float tolerence = 0.6;
int i = 0;
float degree = 0;
String ip = "192.168.1.104";

void setup(){
  frameRate(25);
  size(1207,1000);
  imageMode(CENTER);
  
  //client to send to node server
  client = new Client(this,ip,3000);

  //kinect
  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableUser();
  kinect.enableRGB();
  kinect.setMirror(true);

  //cam
  cam = new ipcapture.IPCapture(this, "http://"+ip+":8081/stream.mjpg", "", "");
  cam.start();
}

void draw(){
  //overdraw steering wheel
  fill(0,0,0);
  ellipse(13+439/2,410+445/2,439,445);

  //kinect image
  kinect.update();
  image(kinect.rgbImage(), 605,655,150,150);

  //camera
  if (cam.isAvailable()) {
    cam.read();
    image(cam,0+1207/2,0+430/2,1207,430);
   }

  image(loadImage("dash.png"), 0 + 1207/2, 300 + 723/2);
  int[] userList = kinect.getUsers();
  for (int i = 0; i < userList.length; ++i) {
    if(kinect.isTrackingSkeleton(userList[i]))
    {
      drawSkelet(userList[i]);
      fill(255,0,0);
      ellipse(600,700,20,20);
    }
  }
  
  translate(13 + 439/2, 410 + 445/2);
  rotate(radians(-degree));
  translate(-(13+439/2),-(410+445/2));
  image(loadImage("steer.png"),13 + 439/2,410 + 445/2);
        
 
}

void drawSkelet(int userId){
  PVector rightHandPos = new PVector();
  float confidenceRight = kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_HAND , rightHandPos);
  PVector projRightHandPos = new PVector();
  if(confidenceRight > tolerence){
    //fill(0,255,0);
    kinect.convertRealWorldToProjective(rightHandPos,projRightHandPos);
    //ellipse(projRightHandPos.x, projRightHandPos.y, 20, 20);
  }

  PVector leftHandPos = new PVector();
  float confidenceLeft = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,leftHandPos);
  PVector projLeftHandPos = new PVector();
  if(confidenceLeft > tolerence){
    //fill(255,0,0);
    kinect.convertRealWorldToProjective(leftHandPos,projLeftHandPos);
    //ellipse(projLeftHandPos.x, projLeftHandPos.y, 20, 20);
  }

  //line(projLeftHandPos.x, projLeftHandPos.y, projRightHandPos.x, projRightHandPos.y);

  float midX=projLeftHandPos.x+((projRightHandPos.x-projLeftHandPos.x)/2.0);
  float midY=projLeftHandPos.y+((projRightHandPos.y-projLeftHandPos.y)/2.0);

  //ellipse(midX, midY, 20, 20);

  //line(midX - 640,midY,midX+640,midY);
  //ellipse(640, midY, 20, 20);

  float a = calculateDistance(640, midY, projLeftHandPos.x, projLeftHandPos.y);
  float b = calculateDistance(midX, midY, projLeftHandPos.x, projLeftHandPos.y);
  float c = calculateDistance(midX, midY, 640, midY);

  float iA = a * a;
  float iB = b * b;
  float iC = c * c;

  float teller = iB + iC - iA;
  float noemer = 2 * b * c;

  float cosA = teller / noemer;
  degree = degrees(acos(cosA));

  if (projLeftHandPos.y > midY) {
    degree = -degree;
  }

  if(i%10 == 0){
    i = 0;
    client.write(degree+"/");
  }
  i++;
}

void onNewUser(SimpleOpenNI kinect, int userId){
  kinect.startTrackingSkeleton(userId);
}

float calculateDistance (float x1, float y1, float x2, float y2) {
  float notC = ((x1-x2) * (x1-x2)) + ((y1-y2) * (y1-y2));
  float c = sqrt(notC);

  return c;
}
