#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofBackground(20); // brigthness of background
    ofSetVerticalSync(true);
    
    bFill       = true;
    bWireframe  = false;
    
    cam.setDistance(4000);
    
    gui.setup(); //panel
    gui.add(width.setup("width",2000,0,10000));
    gui.add(height.setup("heigth",2000,0,10000));
    gui.add(depth.setup("depth",2000,0,10000));
    gui.add(posx.setup("posx",0,-300,300));
    gui.add(posy.setup("posy",0,-300,300));
    gui.add(posz.setup("posz",0,-300,300));
    gui.add(objreal.setup("Real Object",false));
    gui.add(objdiode.setup("Photosensors Object",false));
    gui.add(objaccel.setup("Accelerometer Object",false));
    gui.add(objkalman.setup("Fusion Object",false));
    
    //lighthouse radius
    lightHouse1.setRadius(10);
    lightHouse2.setRadius(10);
    //objects radius
    obj.setRadius(5);
    objCalc.setRadius(5);
    objCalcAccel.setRadius(5);
    objKalman.setRadius(5);
    //normal to the plan vector radius
    b1.setRadius(3);
    b2.setRadius(3);
    b3.setRadius(3);
    b4.setRadius(3);
    
    //object roation
    euler.set(0,0,0);
    
    //Lighthouse A position
    rotationZA=90;
    rotationYA=-45;
    rotationXA=0;
    
    //Lighthouse B position
    rotationZB=90;
    rotationYB=135;
    rotationXB=0;
    
    posCalc.set(0,0,0);



    // init position of the position of the object estimated by the accelerometer
    posAccelX=0;
    posAccelY=0;
    posAccelZ=0;
    
    
    //timer initialisation
    
    timeInit=ofGetElapsedTimeMicros();
    timeInitAccel=ofGetElapsedTimeMicros();
    timeKalman_init=ofGetElapsedTimeMicros();
    //light source
    pointLight.setPosition(-width, width, width);
    pointLight2.setPosition(width, -width, width);
    pointLight3.setPosition(width, width, -width);

    // shininess is a value between 0 - 128, 128 being the most shiny //
    material.setShininess( 120 );
    // the light highlight of the material //
    material.setSpecularColor(ofColor(255, 255, 255, 255));
    
    
    //background
    ofSetSphereResolution(24);
    
    // init Kalman
    


    // Kalman command
    Eigen::MatrixXd C_IMU(3, 9);
    C_IMU <<0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1;
    
    //initial measure
    Eigen::VectorXd measureY(3);
    measureY<< 0,0,0;
    
    //initial state
    Eigen::VectorXd x(9);
    x<<0,0,0,0,0,0,0,0,0;
    
    //kalman initial
    kf.fusionInit(0,measureY,C_IMU,x);
    
    //initial iteration
    i=-1;
}

//--------------------------------------------------------------
void ofApp::update(){
    
    //set position of the real object
    pobj.set(posx,posy,posz);

    //set position of the lighthouses
    p1.set(width/2,height/2,depth/2);
    p2.set(-width/2,height/2,-depth/2);
    
    //get time for the photodiodes
    elaspsedTime=(ofGetElapsedTimeMicros()-timeInit)/1000000.0;
  
    // at the ends of the 4 scanning
    if(elaspsedTime>sampleTime){
        //position calculus
        posCalc=calculPosition(width,height,depth, rotationXA, rotationYA,rotationZA, rotationXB, rotationYB, rotationZB, t1, t2, t3, t4, sampleTime);
        //timer initilise
        timeInit=ofGetElapsedTimeMicros();
        
        //interation
        i++;
        //time initiailisation for the scalar product
        minDot1=99;
        minDot2=99;
        minDot3=99;
        minDot4=99;
        
        //time calculated initialisation
        t1=-1;
        t2=-1;
        t3=-1;
        t4=-1;

        Eigen::MatrixXd C_lh(3,9); // Measurement matrix C_lh : for light house measurement

            C_lh << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, 0, 0;

        //covariance of the sensor
        
        Eigen::MatrixXd R(3,3);
        
        R<< 0.5901, -0.0280,0.3103,
        -0.0280, 0.4582,-0.0732,
        0.3103, -0.0732,0.4489;
        
        //Kalman measurement
        Eigen::VectorXd measureY(3);
        measureY<<posCalc.x,posCalc.y,posCalc.z;
        
        //time update for kalman
        timeKalman=(ofGetElapsedTimeMicros()-timeKalman_init)/1000000.0;
        timeKalman_init=ofGetElapsedTimeMicros();
        
        //kalman update
        kf.fusionUp(timeKalman, measureY,C_lh,R);
        
        // collect datas for the tracking
        TimesR[i]=ofGetElapsedTimeMicros()/1000000.0;
        posXOpt[i]=posCalc.x;
        posYOpt[i]=posCalc.y;
        posZOpt[i]=posCalc.z;
        posXReal[i]=posx;
        posYReal[i]=posy;
        posZReal[i]=posz;
        posXAcc[i]=posAccelX;
        posYAcc[i]=posAccelY;
        posZAcc[i]=posAccelZ;
        posXKalman[i]=kf.state()(0);
        posYKalman[i]=kf.state()(3);
        posZKalman[i]=kf.state()(6);
        

    }
    //scanning numer indicator
    if(elaspsedTime>0.75*sampleTime){
        scanningFlag=4;
    }else if(elaspsedTime>0.5*sampleTime){
        scanningFlag=3;
    }else if(elaspsedTime>0.25*sampleTime){
        scanningFlag=2;
    }else if(elaspsedTime>0){
        scanningFlag=1;
    }
    // flash visualisation
    if(elaspsedTime>0 && elaspsedTime<0.1*sampleTime){
        flashColor1.r=255;
        flashColor1.g=255;
        flashColor1.b=255;
        flashColor2.r=0;
        flashColor2.g=0;
        flashColor2.b=0;
    }else if(elaspsedTime>sampleTime*0.25 && elaspsedTime<(sampleTime*0.25+0.1*sampleTime)){
        flashColor1.r=255;
        flashColor1.g=255;
        flashColor1.b=255;
        flashColor2.r=0;
        flashColor2.g=0;
        flashColor2.b=0;
    }else if(elaspsedTime>sampleTime*0.5 && elaspsedTime<sampleTime*0.5+0.1*sampleTime){
        flashColor1.r=0;
        flashColor1.g=0;
        flashColor1.b=0;
        flashColor2.r=255;
        flashColor2.g=255;
        flashColor2.b=255;
    }else if(elaspsedTime>sampleTime*0.75 && elaspsedTime<sampleTime*0.75+0.1*sampleTime){
        flashColor1.r=0;
        flashColor1.g=0;
        flashColor1.b=0;
        flashColor2.r=255;
        flashColor2.g=255;
        flashColor2.b=255;
    }else{
        flashColor2.r=0;
        flashColor2.g=0;
        flashColor2.b=0;
        flashColor1.r=0;
        flashColor1.g=0;
        flashColor1.b=0;
    }
    // first scanning
    if(scanningFlag==1){
        
        double angle=omega*elaspsedTime;
        //scanning vector calculus
        n1.set(0,cos(angle),-sin(angle));
        // rotation vector obtained from the lighthouse orientation
        n1.rotate(rotationZA, ofVec3f(0,0,1));
        n1.rotate(rotationYA, ofVec3f(0,1,0));
        n1.rotate(rotationXA, ofVec3f(1,0,0));
        
        // lighthouse object vector determination
        plan1=p1-pobj;
        
        //normalizing plan
        plan1.normalize();
        n1.normalize();
        
        //data recovery for the tracking
        dots[i]=abs(n1.dot(plan1));
        TIMES[i]=elaspsedTime;
        
        // minimum scalar product determnation
        if(abs(n1.dot(plan1))<minDot1){
            t1=elaspsedTime;
            minDot1=abs(n1.dot(plan1));
        }
    }else{
        n1.set(0, 0,0);
    }
    if(scanningFlag==2){
        double angle=omega*(elaspsedTime-sampleTime*0.25);
        
        n2.set(-sin(angle),-cos(angle),0);
        n2.rotate(rotationZA, ofVec3f(0,0,1));
        n2.rotate(rotationYA, ofVec3f(0,1,0));
        n2.rotate(rotationXA, ofVec3f(1,0,0));
        
        
        plan2=p1-pobj;
        plan2.normalize();
        n2.normalize();

        
        if(abs(n2.dot(plan2))<minDot2){
            t2=elaspsedTime-sampleTime*0.25;
            minDot2=abs(n2.dot(plan2));
        }
    }else{
        n2.set(0,0,0);
    }
    if(scanningFlag==3){
        
        double angle=omega*(elaspsedTime-sampleTime*0.5);
        
        n3.set(0,cos(angle),-sin(angle));
        n3.rotate(rotationZB, ofVec3f(0,0,1));
        n3.rotate(rotationYB, ofVec3f(0,1,0));
        n3.rotate(rotationXB, ofVec3f(1,0,0));
        
        plan3=p2-pobj;
        plan3.normalize();
        n3.normalize();
        

        if(abs(n3.dot(plan3))<minDot3){
            t3=elaspsedTime-sampleTime*0.5;
            minDot3=abs(n3.dot(plan3));
        }
    }else{
        n3.set(0,0,0);
    }
    if(scanningFlag==4){
        double angle=omega*(elaspsedTime-sampleTime*0.75);
        
        n4.set(-sin(angle),-cos(angle),0);
        n4.rotate(rotationZB, ofVec3f(0,0,1));
        n4.rotate(rotationYB, ofVec3f(0,1,0));
        n4.rotate(rotationXB, ofVec3f(1,0,0));
        
        plan4=p2-pobj;
        plan4.normalize();
        n4.normalize();

        if(abs(n4.dot(plan4))<minDot4){
            t4=elaspsedTime-sampleTime*0.75;
            minDot4=abs(n4.dot(plan4));
        }
    }else{
        n4.set(0,0,0);
        t4=-1;

    }
    
    //timer for the accelerometer
    timeAccel=ofGetElapsedTimeMicros();
    
    // for each time we get the acceleration:
    if (((timeAccel-timeInitAccel)/1000.0)>accelTime){
        
        //iteration
        i++;
        
        //time and position recovery
        time1=time2;
        time2=time3;
        time3=timeAccel/1000.0;
        posx1=posx2;
        posx2=posx3;
        posx3=posx;
        posy1=posy2;
        posy2=posy3;
        posy3=posy;
        posz1=posz2;
        posz2=posz3;
        posz3=posz;
        
        //acceleration calculus
        accelX=calculAccel(posx1, posx2, posx3, time1, time2, time3);
        accelY=calculAccel(posy1, posy2, posy3, time1, time2, time3);
        accelZ=calculAccel(posz1, posz2, posz3, time1, time2, time3);
        
        //initilisation of the timer
        timeInitAccel=ofGetElapsedTimeMicros();
        
        // Kalamn comand for the accelerometer
        Eigen::MatrixXd C_IMU(3, 9); // Measurement matrix C_IMU : for IMU measurement
        
        C_IMU <<0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
        
        //covariance of accelerometer
        Eigen::MatrixXd R(3,3);
        R<< 1000,0,0,
            0,1000,0,
            0,0,1000;
        
        //measure of the accelerometer
        Eigen::VectorXd measureY(3);
        measureY<<accelX,accelY,accelZ;
        
        //Kalamn timer updated
        timeKalman=(ofGetElapsedTimeMicros()-timeKalman_init)/1000000.0;
        timeKalman_init=ofGetElapsedTimeMicros();
        
        //kalman update
        kf.fusionUp(timeKalman, measureY,C_IMU,R);

        //speed obtqiend from the acceleration
        vitesseX=vitesseX+accelX*(time3-time1);
        vitesseY=vitesseY+accelY*(time3-time1);
        vitesseZ=vitesseZ+accelZ*(time3-time1);
        
        //position from the acceleration
        posAccelX=posAccelX+vitesseX*(time2-time1);
        posAccelY=posAccelY+vitesseY*(time2-time1);
        posAccelZ=posAccelZ+vitesseZ*(time2-time1);
        
        
        // datas for tracking
        TimesR[i]=ofGetElapsedTimeMicros()/1000000.0;
        posXOpt[i]=posCalc.x;
        posYOpt[i]=posCalc.y;
        posZOpt[i]=posCalc.z;
        posXReal[i]=posx;
        posYReal[i]=posy;
        posZReal[i]=posz;
        posXAcc[i]=posAccelX;
        posYAcc[i]=posAccelY;
        posZAcc[i]=posAccelZ;
        posXKalman[i]=kf.state()(0);
        posYKalman[i]=kf.state()(3);
        posZKalman[i]=kf.state()(6);
    }
    
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    //luminance
    ofEnableLighting();
    pointLight.enable();
    pointLight2.enable();
    pointLight3.enable();


    // draw the outer sphere
    material.begin();
    ofNoFill();
    ofDrawSphere(ofGetWidth()/2, ofGetHeight()/2, ofGetWidth());
    material.end();
    gui.draw();
    
    cam.begin();

    if(!bFill && bWireframe){
        // if we are only drawing the wireframe, use
        // the material to draw it, otherwise the material
        // will be bound and unbound for every geometry
        // and the wireframe will be drawn in black
        material.begin();
    }

    //dimension de la pieces
    box.setWidth(width);
    box.setHeight(height);
    box.setDepth(depth);


    
    if(bFill) {
        material.begin();
        ofFill();
        box.draw();
        box.setResolution(1);
        material.end();

    }

    if(bWireframe) {
        ofNoFill();
        ofSetColor(0, 0, 0);
        box.setScale(1.01f);
        box.drawWireframe();
        box.setScale(1.f);
    }

  
    if(!bFill && bWireframe){
        material.end();
    }

  
    
    


    //limitation de l'objet reel dans la piece
    if(posx>width*.5)
        posx=width*.5;
    if(posy>height*.5)
        posy=height*.5;
    if(posz>depth*.5)
        posz=depth*.5;
    if(posx<-width*.5)
        posx=-width*.5;
    if(posy<-height*.5)
        posy=-height*.5;
    if(posz<-depth*.5)
        posz=-depth*.5;
    
    ofSetColor(0, 0, 0);
    

    //position de l'objet reel
    obj.setPosition(pobj);
    ofSetColor(180,180,180);
    //obj.drawAxes(obj.getRadius()+30);
    euler.set(rotx, roty,rotz);
    obj.setOrientation(euler);
    if(objreal)
    obj.draw();
    
    objCalc.setPosition(posCalc);
    ofSetColor(0,0,180);
    if(objdiode)
    objCalc.draw();
    
    objCalcAccel.setPosition(posAccelX,posAccelY,posAccelZ);
    ofSetColor(180,0,0);
    if(objaccel)
    objCalcAccel.draw();

    objKalman.setPosition(kf.state()(0),kf.state()(3),kf.state()(6));
    ofSetColor(0,180,0);
    if(objkalman)
    objKalman.draw();
    
    
    //Visualization of scanning plans
    ofSetColor(255, 0, 0);
    b1.setPosition(50*n1.x+lightHouse1.getX(), 50*n1.y+lightHouse1.getY(), 50*n1.z+lightHouse1.getZ());
    b1.draw();
    b2.setPosition(50*n2.x+lightHouse1.getX(), 50*n2.y+lightHouse1.getY(), 50*n2.z+lightHouse1.getZ());
    ofSetColor(255, 0, 0);
    b2.draw();
    b3.setPosition(50*n3.x+lightHouse2.getX(), 50*n3.y+lightHouse2.getY(), 50*n3.z+lightHouse2.getZ());
    ofSetColor(255, 0, 0);
    b3.draw();
    b4.setPosition(50*n4.x+lightHouse2.getX(), 50*n4.y+lightHouse2.getY(), 50*n4.z+lightHouse2.getZ());
    ofSetColor(255, 0, 0);
    b4.draw();

    //light house 1
    lightHouse1.setPosition(width/2., height/2., depth/2.);
    ofSetColor(flashColor1);
    lightHouse1.draw();
    //light house 2
    lightHouse2.setPosition(-width/2., height/2., -depth/2.);
    
    ofSetColor(flashColor2);
    lightHouse2.draw();
    cam.end();
    ofSetColor(255, 255, 255);
    
    
    
    // displays
    
    stringstream ss,ss2,ss3,ss4;
    
    ss<<"Elapsed time: " << elaspsedTime << "\n";
    ofDrawBitmapString(ss.str().c_str(), 20, 20);
    
    ofSetColor(255, 255, 255);
    ss2<<"num of scanning:"<<scanningFlag<<"\n t1="<<t1<<"\n t2="<<t2<<"\n t3="<<t3<<"\n t4="<<t4<<"\n"<<" mindot1="<<minDot1<<"\n mindot2="<<minDot2<<"\n mindot3="<<minDot3<<"\n mindot4="<<minDot4<<"\n";
    ofSetColor(255, 255, 255);
    ofDrawBitmapString(ss2.str().c_str(), 20, 300);
    
    ofSetColor(255, 255, 255);
    ss3<<" accelX="<<accelX<<"\n accelY="<<accelY<<"\n accelZ="<<accelZ<<"\n ";
    ofSetColor(255, 255, 255);
    ofDrawBitmapString(ss3.str().c_str(), 20, 500);
    
    ss4<<" posCalcX="<<posCalc.x<<"\n posCalcY="<<posCalc.y<<"\n posCalcZ="<<posCalc.z<<"\n ";
    ofSetColor(255, 255, 255);
    ofDrawBitmapString(ss4.str().c_str(), 20, 700);
    

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch(key) {
        case 'f':
            ofToggleFullscreen();
            break;
        case 's':
            bFill = !bFill;
            break;
        case 'w':
            bWireframe = !bWireframe;
            break;
    }

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::exit(){
    // recovery of datas for graphes
    ofFile myTempsRFile,myPosXFile,myPosYFile,myPosZFile,myPosAccelXFile,myPosAccelYFile,myPosAccelZFile,myPosCalcXFile,myPosCalcYFile,myPosCalcZFile,myPosKalmanXFile,myPosKalmanYFile,myPosKalmanZFile;
    myTempsRFile.open("tempsR.dat",ofFile::WriteOnly);
    myPosXFile.open("posX.dat",ofFile::WriteOnly);
    myPosYFile.open("posY.dat",ofFile::WriteOnly);
    myPosZFile.open("posZ.dat",ofFile::WriteOnly);
    myPosAccelXFile.open("PosAccelX.dat",ofFile::WriteOnly);
    myPosAccelYFile.open("PosAccelY.dat",ofFile::WriteOnly);
    myPosAccelZFile.open("PosAccelZ.dat",ofFile::WriteOnly);
    myPosCalcXFile.open("PosCalcX.dat",ofFile::WriteOnly);
    myPosCalcYFile.open("PosCalcY.dat",ofFile::WriteOnly);
    myPosCalcZFile.open("PosCalcZ.dat",ofFile::WriteOnly);
    myPosKalmanXFile.open("PosKalmanX.dat",ofFile::WriteOnly);
    myPosKalmanYFile.open("PosKalmanY.dat",ofFile::WriteOnly);
    myPosKalmanZFile.open("PosKalmanZ.dat",ofFile::WriteOnly);
    for (int j=0; j<10000; j++){
        myTempsRFile << TimesR[j]<<" ";
        myPosXFile<<posXReal[j]<<" ";
        myPosYFile<<posYReal[j]<<" ";
        myPosZFile<<posZReal[j]<<" ";
        myPosAccelXFile<< posXAcc[j]<<" ";
        myPosAccelYFile<< posYAcc[j]<<" ";
        myPosAccelZFile<< posZAcc[j]<<" ";
        myPosCalcXFile<< posXOpt[j]<<" ";
        myPosCalcYFile<< posYOpt[j]<<" ";
        myPosCalcZFile<< posZOpt[j]<<" ";
        myPosKalmanXFile<< posXKalman[j]<<" ";
        myPosKalmanYFile<< posYKalman[j]<<" ";
        myPosKalmanZFile<< posZKalman[j]<<" ";
    }
    
}



double calculAccel(double pos1, double pos2, double pos3, double t1, double t2, double t3){
    double accel,err;
    
    accel=0;
    if(t1!=t2 && t2!=t3 && t1!=t3)
        accel=((pos3-pos2)/(t3-t2)-(pos2-pos1)/(t2-t1))/(t3-t1);
    err=0.001*ofRandomf()*accel;//error max is 0.1 percent
    return accel+err;
}


ofVec3f calculPosition(float width, float heigth, float depth, float rotationXA, float rotationYA, float rotationZA, float rotationXB, float rotationYB, float rotationZB, float tA1, float tA2, float tB1, float tB2, float sampleTime)
{
    
    //LightHouseA
    float BoxAx = width / 2;
    float BoxAy = heigth / 2;
    float BoxAz = depth / 2;
    
    //LightHouseB
    float BoxBx = -width / 2;
    float BoxBy = heigth / 2;
    float BoxBz = -depth / 2;
    
    // Emission parametters of Light House
    float period = sampleTime/4; // half turn
    
    // Find Light House A position angle
    float gammaA = Gamma(tA1, period);
    float thetaA = Theta(tA2, period);
    // Find Light House A position angle
    float gammaB = Gamma(tB1, period);
    float thetaB = Theta(tB2, period);
    
    /**
     *  Lighthouse vector calculation to object in spherical coordinates
     **/
    
    //LighthouseA
    
    vector<float> posA;
    
    posA = posFromLightHouse(gammaA, thetaA);
    float xA = posA[0]; float yA = posA[1]; float zA = posA[2];
    
    // Landmark change to return to the world landmark
    ofVec3f v;
    v.set(xA, yA, zA);
    v.rotate(0, 0, rotationZA); // rotation around the Z axis
    v.rotate(0, rotationYA, 0); // rotation around the Y axis
    v.rotate(rotationXA, 0, 0); // rotation around the X axis
    
    // translation
    v = v + ofVec3f(BoxAx, BoxAy, BoxAz);
    
    
    //LightHouseB
    vector<float> posB;
    posB = posFromLightHouse(gammaB, thetaB);
    float xB = posB[0]; float yB = posB[1]; float zB = posB[2];
    
    // Landmark change to return to the world landmark
    ofVec3f w;
    w.set(xB, yB, zB);
    w.rotate(0, 0, rotationZB);
    w.rotate(0, rotationYB, 0);
    w.rotate(rotationXB, 0, 0);
    
    // translation
    w = w + ofVec3f(BoxBx, BoxBy, BoxBz);
    
    //Assignment of parameters
    xA = v.x; xB = w.x; yA = v.y; yB = w.y; zA = v.z; zB = w.z;
    
    // Calculating Parametric Equations of Two Straight
    vector <float> a;
    a.push_back(xA - BoxAx);
    a.push_back(yA - BoxAy);
    a.push_back(zA - BoxAz);
    vector <float> b;
    b.push_back(xB - BoxBx);
    b.push_back(yB - BoxBy);
    b.push_back(zB - BoxBz);
    vector <float> c(3);
    c[0] = BoxBx - BoxAx;
    c[1] = BoxBy - BoxAy;
    c[2] = BoxBz - BoxAz;
    
    /*
     Using Imperfect intersection approximation Method
     to find intersection of the Two straights
     */
    float u2;
    u2 = (pow(a[0], 2) + pow(a[1], 2)+ pow(a[2], 2));    // Calcul U*U
    float v2;
    v2 = (pow(b[0], 2) + pow(b[1], 2) + pow(b[2], 2));    // Calcul V*V
    float uv;
    uv = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];        // Calcul U*V
    float ABu;
    ABu = (BoxBx - BoxAx)*a[0] + (BoxBy - BoxAy)*a[1] + (BoxBz - BoxAz)*a[2];
    float ABv;
    ABv = (BoxBx - BoxAx)*b[0] + (BoxBy - BoxAy)*b[1] + (BoxBz - BoxAz)*b[2];
    
    // Resolution of the equation system
    vector<float> lamda_mu;
    lamda_mu = equationSystem(u2, -uv, ABu, uv, -v2, ABv);
    vector<float> XYZ(3);
    XYZ[0] = (BoxAx + BoxBx + a[0] * lamda_mu[0] + b[0] * lamda_mu[1])/2;
    XYZ[1] = (BoxAy + BoxBy + a[1] * lamda_mu[0] + b[1] * lamda_mu[1])/2;
    XYZ[2] = (BoxAz + BoxBz + a[2] * lamda_mu[0] + b[2] * lamda_mu[1])/2;
    
    // Limits controls
    XYZ = controLimit(XYZ, width, heigth, depth);
    // Return object position
    return ofVec3f( XYZ[0], XYZ[1], XYZ[2]);
    
}




