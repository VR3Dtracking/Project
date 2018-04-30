#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "functions.h"
#include "kalman.hpp"
#include <vector>
#include <Eigen/Dense>

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();
    
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
        bool bFill;
        bool bWireframe;
    
        //system time
        float elaspsedTime;
        //elapsed time on the photdiode
        float t1=-1,t2=-1,t3=-1,t4=-1;
    
        // total time for the 4 sweeping
        double sampleTime= 10;
    
        //timers
        double timeInit,timeInitAccel,timeAccel;
    
        // Scanning flag number
        int scanningFlag=0;
        // Rotation spped for the lighthouse
        double omega=(4*pi)/sampleTime;
    
        //minimum scalar product between the lighthouse-object vector and the normal to the plan vector
        double minDot1=99,minDot2=99,minDot3=99,minDot4=99;
        //time and position for the acceleration
        double time1=0,time2=0,time3=0;
        double posx1=0,posx2=0,posx3=0;
        double posy1=0,posy2=0,posy3=0;
        double posz1=0,posz2=0,posz3=0;
        // Acceleration and computed speed
        double accelX, accelY, accelZ, vitesseX=0, vitesseY=0, vitesseZ=0;
    
        //acceleration frequence time calculus (in second)
        double accelTime=sampleTime/3;
    
        // .dat files for the graphics
        float dots[100000];
        float TIMES[100000];
        float TimesR[100000];
        float posXOpt[100000];
        float posYOpt[100000];
        float posZOpt[100000];
        float posXReal[100000];
        float posYReal[100000];
        float posZReal[100000];
        float posXAcc[100000];
        float posYAcc[100000];
        float posZAcc[100000];
        float posXKalman[100000];
        float posYKalman[100000];
        float posZKalman[100000];
    
        //iteration value
        int i=0;
    
        //rotation of lighthouses
        double rotationZA,rotationYA,rotationXA,rotationZB,rotationYB,rotationXB;
    
        //time for Kalman calculus
        double timeKalman,timeKalman_init;
        double posAccelX=0,posAccelY=0,posAccelZ=0;

    
    
        // Panel parameters
        ofxPanel gui;
        ofxIntSlider width;
        ofxIntSlider height;
        ofxIntSlider depth;
        ofxIntSlider posx;
        ofxIntSlider posy;
        ofxIntSlider posz;
        ofxIntSlider rotx;
        ofxIntSlider roty;
        ofxIntSlider rotz;
    
        //object orientation
        ofVec3f euler;
        //position obtainted with the lighthouse
        ofVec3f posCalc;

        // light of the room
        ofLight pointLight;
        ofLight pointLight2;
        ofLight pointLight3;
    
    
        ofMaterial material;
        ofEasyCam cam;
    
        // the room
        ofBoxPrimitive box;

        //normal to the plan vector
        ofVec3f n1,n2,n3,n4;
    
        //lighthouse and object position
        ofVec3f p1,p2,pobj;
    
        //vector in the lighthouse-object plan
        ofVec3f plan1,plan2,plan3,plan4;
    
        // lighthouse representation
        ofSpherePrimitive lightHouse1;
        ofSpherePrimitive lightHouse2;
    
        // balls representing the object and the plan
        ofSpherePrimitive obj,b1,b2,b3,b4;
    
        //ball representing the estimated object zith different methods
        ofSpherePrimitive objCalc,objKalman,objCalcAccel;
    
        //lighthouse flashes
        ofColor flashColor1;
        ofColor flashColor2;
    
    
        //toggles
        ofxToggle objreal;
        ofxToggle objdiode;
        ofxToggle objaccel;
        ofxToggle objkalman;
        // Kalman filter
        KalmanFilter kf;

    
};


double calculAccel(double pos1, double pos2, double pos3, double t1, double t2, double t3);


ofVec3f calculPosition(float width, float heigth, float depth, float rotationXA, float rotationYA, float rotationZA, float rotationXB, float rotationYB, float rotationZB, float tA1, float tA2, float tB1, float tB2, float sampleTime);
