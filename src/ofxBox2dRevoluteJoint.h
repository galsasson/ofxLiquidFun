//
//  ofxBox2dRevoluteJoint.h
//  liquidfun
//
//  Created by Gal Sasson on 2/2/15.
//
//

#ifndef __liquidfun__ofxBox2dRevoluteJoint__
#define __liquidfun__ofxBox2dRevoluteJoint__

#include <stdio.h>
#pragma once
#include "ofMain.h"
#include "Box2D.h"
#include "ofxBox2dUtils.h"

#define BOX2D_DEFAULT_FREQ      4.0
#define BOX2D_DEFAULT_DAMPING   0.5

class ofxBox2dRevoluteJoint {

public:

	b2World			*	world;
	b2RevoluteJoint *	joint;
	int					jointType;
	bool				alive;

	//----------------------------------------
	ofxBox2dRevoluteJoint();
	ofxBox2dRevoluteJoint(b2World* b2world, b2Body* body1, b2Body* body2, float frequencyHz=4.f, float damping=.5f, bool bCollideConnected=true);
	ofxBox2dRevoluteJoint(b2World* b2world, b2Body* body1, b2Body* body2, ofVec2f anchor1, ofVec2f anchor2, float frequencyHz=4.f, float damping=.5f, bool bCollideConnected=true);

	//----------------------------------------
	void setWorld(b2World * w);
	void setup(b2World* b2world, b2Body* body1, b2Body* body2, float frequencyHz=4.f, float damping=.5f, bool bCollideConnected=true);
	void setup(b2World* b2world, b2Body* body1, b2Body* body2, ofVec2f anchor1, ofVec2f anchor2, float frequencyHz=4.f, float damping=.5f, bool bCollideConnected=true);
//	void setup(b2World* b2world, b2Body* body1, b2Body* body2, ofVec2f anchor, float frequencyHz=4.f, float damping=.5f, bool bCollideConnected=true);


	void setLimits(float lowDeg, float highDeg);
	void enableLimits(bool enable);

	void setMotorSpeed(float speed);
	void setMaxMotorTorque(float torque);
	void enableMotor(bool enable);

	//----------------------------------------
	bool isSetup();
	void draw();
	void destroy();

	//----------------------------------------
	ofVec2f getReactionForce(float inv_dt) const;
	b2Vec2  getReactionForceB2D(float inv_dt) const;
	float   getReactionTorque(float inv_dt) const;
};













#endif /* defined(__liquidfun__ofxBox2dRevoluteJoint__) */
