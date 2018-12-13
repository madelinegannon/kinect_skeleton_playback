#pragma once


#include "ofMain.h"
#include "ofxJSON.h"


class ofApp: public ofBaseApp
{
public:
    void setup();
	void update();
    void draw();

    ofxJSONElement result;
	string filename = "example_kinectron_skeleton_long.json";
	//string filename = "example_kinectron_skeleton_posing.json";
	bool convert_mm = true;
	bool convert_world_coords = true;

	ofEasyCam cam;
	void draw_3D();
	int counter = 0;
	int playhead = 0;

	void update_playhead(int start_frame, int end_frame);

	struct Body {

		int id, timestamp;
		string id_tracking; 

		bool is_tracked, hand_state_L, hand_state_R;

		vector<ofNode> joints;

		// joint index ordering from Kinectron {https://kinectron.github.io/docs/api2.html}
		enum {
			SPINE_BASE,
			SPINE_MID,
			NECK,
			HEAD,
			SHOULDER_LEFT,
			ELBOW_LEFT,
			WRIST_LEFT,
			HAND_LEFT,
			SHOULDER_RIGHT,
			ELBOW_RIGHT,
			WRIST_RIGHT,
			HAND_RIGHT,
			HIP_LEFT,
			KNEE_LEFT ,
			ANKLE_LEFT,
			FOOT_LEFT,
			HIP_RIGHT,
			KNEE_RIGHT,
			ANKLE_RIGHT,
			FOOT_RIGHT,
			SPINE_SHOULDER,
			HAND_TIP_LEFT,
			THUMB_LEFT,
			HAND_TIP_RIGHT,
			THUMB_RIGHT
		};
			
	}; vector<Body*> bodies;
	void draw_body(int id);
	string body_toString(int frame_id);
};
