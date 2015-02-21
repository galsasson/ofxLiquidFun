//
//  ofxBox2dRevoluteJoint.cpp
//  liquidfun
//
//  Created by Gal Sasson on 2/2/15.
//
//

#include "ofxBox2dRevoluteJoint.h"

//----------------------------------------
ofxBox2dRevoluteJoint::ofxBox2dRevoluteJoint() {
	world = NULL;
	joint = NULL;
	alive = false;
}

//----------------------------------------
ofxBox2dRevoluteJoint::ofxBox2dRevoluteJoint(b2World* b2world, b2Body* body1, b2Body* body2, float frequencyHz, float damping, bool bCollideConnected) {
	ofxBox2dRevoluteJoint();
	setup(b2world, body1, body2, frequencyHz, damping, bCollideConnected);
}

//----------------------------------------
ofxBox2dRevoluteJoint::ofxBox2dRevoluteJoint(b2World* b2world, b2Body* body1, b2Body* body2, ofVec2f anchor1, ofVec2f anchor2, float frequencyHz, float damping, bool bCollideConnected) {
	ofxBox2dRevoluteJoint();
	setup(b2world, body1, body2, anchor1, anchor2, frequencyHz, damping, bCollideConnected);
}

//----------------------------------------
void ofxBox2dRevoluteJoint::setup(b2World* b2world, b2Body* body1, b2Body* body2, float frequencyHz, float damping, bool bCollideConnected) {

	if(body1 == NULL || body2 == NULL) {
		ofLog(OF_LOG_NOTICE, "ofxBox2dRevoluteJoint :: setup : - box2d body is NULL -");
		return;
	}

	ofVec2f a1 = worldPtToscreenPt(body1->GetWorldCenter());
	ofVec2f a2 = worldPtToscreenPt(body2->GetWorldCenter());

	setup(b2world, body1, body2, a1, a2, frequencyHz, damping, bCollideConnected);

	alive = true;
}

//----------------------------------------
void ofxBox2dRevoluteJoint::setup(b2World* b2world, b2Body* body1, b2Body* body2, ofVec2f anchor1, ofVec2f anchor2, float frequencyHz, float damping, bool bCollideConnected) {

	setWorld(b2world);

	if(body1 == NULL || body2 == NULL) {
		ofLog(OF_LOG_NOTICE, "ofxBox2dRevoluteJoint :: setup : - box2d body is NULL -");
		return;
	}


	b2RevoluteJointDef jointDef;
	jointDef.bodyA = body1;
	jointDef.bodyB = body2;
	jointDef.localAnchorA = screenPtToWorldPt(anchor1);
	jointDef.localAnchorB = screenPtToWorldPt(anchor2);
	jointDef.referenceAngle = body2->GetAngle() - body1->GetAngle();
	jointDef.collideConnected = false;
	jointDef.enableLimit = false;
	jointDef.enableMotor = true;
	joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);

//
//	b2DistanceJointDef jointDef;
//	jointDef.Initialize(body1, body2, screenPtToWorldPt(anchor1), screenPtToWorldPt(anchor2));
//	jointDef.collideConnected	= bCollideConnected;
//	jointDef.frequencyHz		= frequencyHz;
//	jointDef.dampingRatio		= damping;
//	joint						= (b2DistanceJoint*)world->CreateJoint(&jointDef);

	alive						= true;
}

//----------------------------------------
void ofxBox2dRevoluteJoint::setWorld(b2World* w) {
	if(w == NULL) {
		ofLog(OF_LOG_NOTICE, "ofxBox2dRevoluteJoint :: setWorld : - box2d world needed -");
		return;
	}
	world = w;
}


//----------------------------------------
void ofxBox2dRevoluteJoint::setLimits(float low, float high) {
	joint->SetLimits(ofDegToRad(low), ofDegToRad(high));
}

//----------------------------------------
void ofxBox2dRevoluteJoint::enableLimits(bool enable)
{
	joint->EnableLimit(enable);
}

//----------------------------------------
void ofxBox2dRevoluteJoint::setMotorSpeed(float speed)
{
	joint->SetMotorSpeed(speed);
}

//----------------------------------------
void ofxBox2dRevoluteJoint::setMaxMotorTorque(float torque)
{
	joint->SetMaxMotorTorque(torque);
}

//----------------------------------------
void ofxBox2dRevoluteJoint::enableMotor(bool enable)
{
	joint->EnableMotor(enable);
}


//----------------------------------------
bool ofxBox2dRevoluteJoint::isSetup() {
	if (world == NULL) {
		ofLog(OF_LOG_NOTICE, "ofxBox2dRevoluteJoint :: world must be set!");
		return false;
	}
	if (joint == NULL) {
		ofLog(OF_LOG_NOTICE, "ofxBox2dRevoluteJoint :: setup function must be called!");
		return false;
	}
	return true;
}


//----------------------------------------
void ofxBox2dRevoluteJoint::draw() {
	if(!alive) return;

	b2Vec2 p1 = joint->GetAnchorA();
	b2Vec2 p2 = joint->GetAnchorB();

	p1 *= OFX_BOX2D_SCALE;
	p2 *= OFX_BOX2D_SCALE;
	ofDrawLine(p1.x, p1.y, p2.x, p2.y);
}

//----------------------------------------
void ofxBox2dRevoluteJoint::destroy() {
	if (!isSetup()) return;
	if(joint) {
		world->DestroyJoint(joint);
	}
	joint = NULL;
	alive = false;
}


//----------------------------------------
ofVec2f ofxBox2dRevoluteJoint::getReactionForce(float inv_dt) const {
	b2Vec2 vec = getReactionForceB2D(inv_dt);
	return ofVec2f(vec.x, vec.y);
}
b2Vec2 ofxBox2dRevoluteJoint::getReactionForceB2D(float inv_dt) const {
	if(joint) {
		return joint->GetReactionForce(inv_dt);
	}
	return b2Vec2(0, 0);
}
float ofxBox2dRevoluteJoint::getReactionTorque(float inv_dt) const {
	if(joint) {
		return (float)joint->GetReactionTorque(inv_dt);
	}
	return 0;
}




