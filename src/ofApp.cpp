#include "ofApp.h"


void ofApp::setup()
{
	ofSetFrameRate(45);
  
    // Now parse the JSON
    bool parsingSuccessful = result.open(filename);

    if (parsingSuccessful)
    {
 
		// parse into bodies
		if (result.isArray()) {

			for (Json::ArrayIndex i = 0; i < result.size(); ++i) {

				const Json::Value& id = result[i]["bodyIndex"];
				const Json::Value& id_tracking = result[i]["trackingId"];
				const Json::Value& is_tracked = result[i]["tracked"];
				const Json::Value& timestamp = result[i]["record_timestamp"];
				const Json::Value& hand_state_L = result[i]["leftHandState"];
				const Json::Value& hand_state_R = result[i]["rightHandState"];

				const Json::Value& joints = result[i]["joints"];

				vector<ofNode> temp;
				float scalar;
				if (convert_mm)
					scalar = 1000;
				else
					scalar = 1;
				for (Json::ArrayIndex j = 0; j < joints.size(); ++j)
				{

					float x = joints[j]["cameraX"].asFloat() * scalar;
					float y = joints[j]["cameraY"].asFloat() * scalar;
					float z = joints[j]["cameraZ"].asFloat() * scalar;
					float qx = joints[j]["orientationX"].asFloat();
					float qy = joints[j]["orientationY"].asFloat();
					float qz = joints[j]["orientationZ"].asFloat();
					float qw = joints[j]["orientationW"].asFloat();

					ofNode joint;
					joint.setGlobalOrientation(ofQuaternion(qx, qy, qz, qw));
					joint.setGlobalPosition(ofVec3f(x, y, z));

					if (convert_world_coords) {
						// convert to from kinect to robot's coordinate system
						// Kinect camera space coordinates: https://medium.com/@lisajamhoury/understanding-kinect-v2-joints-and-coordinate-system-4f4b90b9df16
						joint.pan(90); //rotate 90 about the Y
						joint.tilt(90); //rotate 90 about the X
					}

					temp.push_back(joint);
				}

				bodies.push_back(new Body());
				bodies.back()->id = id.asInt();
				bodies.back()->id_tracking = id_tracking.asString();
				bodies.back()->is_tracked = is_tracked.asBool();
				bodies.back()->timestamp = timestamp.asInt();
				bodies.back()->hand_state_L = hand_state_L.asBool();
				bodies.back()->hand_state_R = hand_state_R.asBool();
				for (auto & j : temp)
					bodies.back()->joints.push_back(j);
			}

		}


    }
    else
    {
        ofLogError("ofApp::setup")  << "Failed to parse JSON" << endl;
    }

	// quickly check that everything is loaded properly
	cout << body_toString(0) << endl;
}

void ofApp::update()
{
}


void ofApp::draw()
{
    ofBackground(0);
	if (filename == "example_kinectron_skeleton_long.json")
		update_playhead(110, bodies.size() - 50);
	else if (filename == "example_kinectron_skeleton_posing.json")
		update_playhead(200, 850);
	else 
		update_playhead(0, bodies.size() - 1);
	draw_3D();

    ofDrawBitmapStringHighlight("Frame Rate "+ ofToString(ofGetFrameRate()), ofGetWidth()-165, 10);
	ofDrawBitmapStringHighlight("Frame Counter: "+ofToString(playhead), ofGetWidth() - 165, 25);
}

void ofApp::draw_3D()
{
	ofEnableDepthTest();
	cam.begin();
	ofDrawAxis(150);
	ofPushMatrix();
	draw_body(playhead);
	ofPopMatrix();
	cam.end();
	ofDisableDepthTest();
}

void ofApp::update_playhead(int start_frame, int end_frame)
{
	int frame_count = end_frame - start_frame;

	counter = (counter + 1) % frame_count;
	playhead = ofMap(counter, 0, frame_count, start_frame, end_frame, true);
}

void ofApp::draw_body(int id)
{
	ofPushStyle();
	ofNoFill();
	if (id < bodies.size() - 1) {
		for (int i = 0; i < bodies[id]->joints.size(); i++) {

			// ignore the hand nodes for now
			if (i != Body::HAND_TIP_LEFT  && i != Body::THUMB_LEFT &&
				i != Body::HAND_TIP_RIGHT && i != Body::THUMB_RIGHT ) {

				if (i == Body::HAND_LEFT) {
					if (bodies[id]->hand_state_L == 1)
						ofSetColor(ofColor::magenta);
					else
						ofSetColor(ofColor::white);
				}
				else if (i == Body::HAND_RIGHT) {
					if (bodies[id]->hand_state_R == 1)
						ofSetColor(ofColor::magenta);
					else
						ofSetColor(ofColor::white);
				}
				else {
					ofSetColor(ofColor::white);
				}

				ofDrawBox(bodies[id]->joints[i].getGlobalPosition(), 50 / 1.);
				bodies[id]->joints[i].draw();

			}
		}

		ofSetLineWidth(5);
		ofSetColor(200, 100);
		// draw spine
		ofDrawLine(bodies[id]->joints[Body::SPINE_BASE].getGlobalPosition(), bodies[id]->joints[Body::SPINE_MID].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::SPINE_MID].getGlobalPosition(), bodies[id]->joints[Body::NECK].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::NECK].getGlobalPosition(), bodies[id]->joints[Body::HEAD].getGlobalPosition());
		// draw left arm
		ofDrawLine(bodies[id]->joints[Body::SPINE_SHOULDER].getGlobalPosition(), bodies[id]->joints[Body::SHOULDER_LEFT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::SHOULDER_LEFT].getGlobalPosition(), bodies[id]->joints[Body::ELBOW_LEFT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::ELBOW_LEFT].getGlobalPosition(), bodies[id]->joints[Body::WRIST_LEFT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::WRIST_LEFT].getGlobalPosition(), bodies[id]->joints[Body::HAND_LEFT].getGlobalPosition());
		// draw right arm
		ofDrawLine(bodies[id]->joints[Body::SPINE_SHOULDER].getGlobalPosition(), bodies[id]->joints[Body::SHOULDER_RIGHT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::SHOULDER_RIGHT].getGlobalPosition(), bodies[id]->joints[Body::ELBOW_RIGHT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::ELBOW_RIGHT].getGlobalPosition(), bodies[id]->joints[Body::WRIST_RIGHT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::WRIST_RIGHT].getGlobalPosition(), bodies[id]->joints[Body::HAND_RIGHT].getGlobalPosition());
		// draw left leg
		ofDrawLine(bodies[id]->joints[Body::SPINE_BASE].getGlobalPosition(), bodies[id]->joints[Body::HIP_LEFT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::HIP_LEFT].getGlobalPosition(), bodies[id]->joints[Body::KNEE_LEFT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::KNEE_LEFT].getGlobalPosition(), bodies[id]->joints[Body::ANKLE_LEFT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::ANKLE_LEFT].getGlobalPosition(), bodies[id]->joints[Body::FOOT_LEFT].getGlobalPosition());
		// draw right leg
		ofDrawLine(bodies[id]->joints[Body::SPINE_BASE].getGlobalPosition(), bodies[id]->joints[Body::HIP_RIGHT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::HIP_RIGHT].getGlobalPosition(), bodies[id]->joints[Body::KNEE_RIGHT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::KNEE_RIGHT].getGlobalPosition(), bodies[id]->joints[Body::ANKLE_RIGHT].getGlobalPosition());
		ofDrawLine(bodies[id]->joints[Body::ANKLE_RIGHT].getGlobalPosition(), bodies[id]->joints[Body::FOOT_RIGHT].getGlobalPosition());

	}
	else {
		ofLogNotice() << "invalid id: [" << id << "]. There are only " << bodies.size() << "frames recorded." << endl;
	}
	ofPopStyle();
}

string ofApp::body_toString(int frame_id)
{
	stringstream ss;
	ss << "Body " << frame_id << ":" << endl;
	ss << "\id: " << bodies[frame_id]->id << endl;
	ss << "\id_tracking: " << bodies[frame_id]->id_tracking << endl;
	ss << "\is_tracked: " << bodies[frame_id]->is_tracked << endl;
	ss << "\tNumber of Joints: " << bodies[frame_id]->joints.size() << endl;
	return ss.str();
}
