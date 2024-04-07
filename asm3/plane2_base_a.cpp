//|___________________________________________________________________
//!
//! \file plane2_base.cpp
//!
//! \brief Base source code for the second plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard inputs for plane and propeller (subpart):
//!   s   = moves the plane forward
//!   f   = moves the plane backward
//!   q,e = rolls the plane
//!   a   = yaws the plane
//!   x   = pitches the plane
//!
//!   r   = rotates propeller
//!
//! Mouse inputs for world-relative camera:
//!   Hold left button and drag  = controls azimuth and elevation 
//!                                (Press CTRL (and hold) before left button to restrict to azimuth control only,
//!                                 Press SHIFT (and hold) before left button to restrict to elevation control only)   
//!   Hold right button and drag = controls distance
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

//|___________________
//|
//| Includes
//|___________________

#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

//|___________________
//|
//| Constants
//|___________________

// preset colours
float colour_brown[3] = { 0.45f, 0.32f, 0.22f };
float colour_lime_green[3] = { 0.35f, 0.47f, 0.10f };
float colour_light_lime_green[3] = { 0.45f, 0.57f, 0.20f };
float colour_dark_gray[3] = { 0.2f, 0.2f, 0.2f };
float colour_light_pink[3] = { 0.87f, 0.66f, 0.66f };

// Plane dimensions
const float P_WIDTH = 3;
const float P_LENGTH = 3;
const float P_HEIGHT = 1.5f;

// Plane transforms
const gmtl::Vec3f PLANE_FORWARD(0, 0, 1.0f);            // Plane's forward translation vector (w.r.t. local frame)
const float PLANE_ROTATION = 5.0f;                      // Plane rotated by 5 degs per input

// Propeller dimensions (subpart)
const float PP_WIDTH = 0.25f;
const float PP_LENGTH = 1.5f;

// Propeller transforms
const gmtl::Point3f PROPELLER_POS(P_WIDTH / 4, 0, 0);     // Propeller position on the plane (w.r.t. plane's frame)
const float PROPELLER_ROTATION = 5.0f;                  // Propeller rotated by 5 degs per input

// Camera's view frustum 
const float CAM_FOV = 90.0f;                     // Field of view in degs

// Keyboard modifiers
enum KeyModifier { KM_SHIFT = 0, KM_CTRL, KM_ALT };

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width = 800;
int w_height = 600;

// Plane pose (position-quaternion pair)
gmtl::Point4f plane_p;      // Position for plane 2 (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q;        // Quaternion for plane 2

gmtl::Point4f plane_p1;      // Position for plane 1 (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q1;        // Quaternion for plane 1

// Quaternions to rotate plane
gmtl::Quatf zrotp_q;        // Positive and negative Z rotations
gmtl::Quatf zrotn_q;

gmtl::Quatf xrotp_q;        // Positive and negative Z rotations
gmtl::Quatf xrotn_q;

gmtl::Quatf yrotp_q;
gmtl::Quatf yrotn_q;

// Propeller rotation (subpart)
float pp_angle = 0;         // Rotation angle
float pp_angle_left = 0;	// Rotation angle for the left propeller (new)
float pp_angle_top = 0;	// top propeller
float pp_angle_ss = 0; // subsub part propeller

// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3] = { false, false, false };
bool kmodifiers[3] = { false, false, false };

// Cameras
int cam_id = 0;                                // Selects which camera to view
int camctrl_id = 0;                                // Selects which camera to control
float distance[3] = { 20.0f,  20.0f,  20.0f };                 // Distance of the camera from world's origin.
float elevation[3] = { -45.0f, -45.0f, -45.0f };                 // Elevation of the camera. (in degs)
float azimuth[3] = { 15.0f,  15.0f,  15.0f };                 // Azimuth of the camera. (in degs)

//|___________________
//|
//| Function Prototypes
//|___________________

void InitTransforms();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawTurtleShell(const float width, const float length, const float height);
void DrawPropeller(const float width, const float length);

//|____________________________________________________________________
//|
//| Function: InitTransforms
//|
//! \param None.
//! \return None.
//!
//! Initializes all the transforms
//|____________________________________________________________________

void InitTransforms()
{
	const float COSTHETA_D2 = cos(gmtl::Math::deg2Rad(PLANE_ROTATION / 2));  // cos() and sin() expect radians 
	const float SINTHETA_D2 = sin(gmtl::Math::deg2Rad(PLANE_ROTATION / 2));

	// Inits plane 2 pose
	plane_p.set(1.0f, 0.0f, 4.0f, 1.0f);
	plane_q.set(0, 0, 0, 1);

	// Inits plane 1 pose
	plane_p1.set(-1.0f, 5.0f, 4.0f, 1.0f);
	plane_q1.set(0, 0, 0, 1);

	// Z rotations (roll)
	zrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
	zrotn_q = gmtl::makeConj(zrotp_q);                // -Z

	// X rotation (pitch)
	xrotp_q.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X
	xrotn_q = gmtl::makeConj(xrotp_q);                // -Z

	// Y rotation (yaw)
	yrotp_q.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
	yrotn_q = gmtl::makeConj(yrotp_q);                // -Z

	// TODO: Initializes the remaining transforms
}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
	glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
	gmtl::AxisAnglef aa;    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	gmtl::Vec3f axis;       // Axis component of axis-angle representation
	float angle;            // Angle component of axis-angle representation

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(CAM_FOV, (float)w_width / w_height, 0.1f, 1000.0f);     // Check MSDN: google "gluPerspective msdn"

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//|____________________________________________________________________
	//|
	//| Setting up view transform by:
	//| "move up to the world frame by composing all of the (inverse) transforms from the camera up to the world node"
	//|____________________________________________________________________

	switch (cam_id) {
	case 0:
		// For the world-relative camera
		glTranslatef(0, 0, -distance[0]);
		glRotatef(-elevation[0], 1, 0, 0);
		glRotatef(-azimuth[0], 0, 1, 0);
		break;

	case 1:
		// For plane2's camera
		glTranslatef(0, 0, -distance[1]);
		glRotatef(-elevation[1], 1, 0, 0);
		glRotatef(-azimuth[1], 0, 1, 0);

		gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
		axis = aa.getAxis();
		angle = aa.getAngle();
		glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
		glTranslatef(-plane_p[0], -plane_p[1], -plane_p[2]);
		break;

		// TODO: Add case for the plane1's camera
	case 2:
		// For plane2's camera
		glTranslatef(0, 0, -distance[2]);
		glRotatef(-elevation[2], 1, 0, 0);
		glRotatef(-azimuth[2], 0, 1, 0);

		gmtl::set(aa, plane_q1);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
		axis = aa.getAxis();
		angle = aa.getAngle();
		glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
		glTranslatef(-plane_p1[0], -plane_p1[1], -plane_p1[2]);
		break;
	}

	//|____________________________________________________________________
	//|
	//| Draw traversal begins, start from world (root) node
	//|____________________________________________________________________

	  // World node: draws world coordinate frame
	DrawCoordinateFrame(10);

	// World-relative camera:
	if (cam_id != 0) {
		glPushMatrix();
		glRotatef(azimuth[0], 0, 1, 0);
		glRotatef(elevation[0], 1, 0, 0);
		glTranslatef(0, 0, distance[0]);
		DrawCoordinateFrame(1);
		glPopMatrix();
	}

	// Plane 2 body:
	glPushMatrix();
	gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	axis = aa.getAxis();
	angle = aa.getAngle();
	glTranslatef(plane_p[0], plane_p[1], plane_p[2]);
	glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
	DrawTurtleShell(P_WIDTH, P_LENGTH, P_HEIGHT); // turtle plane base
	DrawCoordinateFrame(3);

	// Plane 2's camera:
	glPushMatrix();
	glRotatef(azimuth[1], 0, 1, 0);
	glRotatef(elevation[1], 1, 0, 0);
	glTranslatef(0, 0, distance[1]);
	DrawCoordinateFrame(1);
	glPopMatrix();

	// Right Propeller (original) (subpart A):
	glPushMatrix();
	glTranslatef(PROPELLER_POS[0], PROPELLER_POS[1], PROPELLER_POS[2]);     // Positions propeller on the plane
	glRotatef(pp_angle, 0, 0, 1);                                           // Rotates propeller
	DrawPropeller(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(1);
	glPopMatrix();

	// Left Propeller (subpart B):
	glPushMatrix();
	glTranslatef(-PROPELLER_POS[0], PROPELLER_POS[1], PROPELLER_POS[2]);     // Positions propeller on the plane
	glRotatef(pp_angle_left, 0, 0, 1);                                           // Rotates propeller
	DrawPropeller(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(1);
	glPopMatrix();

	// Top/Wing Propeller (subpart C):
	glPushMatrix();
	glTranslatef(0, P_HEIGHT, -P_LENGTH / 2);     // Positions propeller on the plane
	glRotatef(pp_angle_top, 0, 0, 1);         // Rotates propeller   
	DrawPropeller(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(1);

	// Top/Wing Propeller (subpart C):
	glPushMatrix();
	glTranslatef(0, PP_LENGTH / 2, 0);     // Positions propeller at the top
	glRotatef(pp_angle_ss, 0, 1, 0);         // Rotates propeller   
	glRotatef(-90, 1, 0, 0);         // Rotates propeller   
	DrawPropeller(PP_WIDTH, PP_LENGTH);
	DrawCoordinateFrame(1);
	glPopMatrix();

	glPopMatrix();
	glPopMatrix();

	// Plane 1 body:
	glPushMatrix();
	gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	axis = aa.getAxis();
	angle = aa.getAngle();
	glTranslatef(plane_p1[0], plane_p1[1], plane_p1[2]);
	glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
	DrawTurtleShell(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(3);

	// Plane 1's camera:
	glPushMatrix();
	glRotatef(azimuth[2], 0, 1, 0);
	glRotatef(elevation[2], 1, 0, 0);
	glTranslatef(0, 0, distance[2]);
	DrawCoordinateFrame(1);
	glPopMatrix();

	glPopMatrix();

	glutSwapBuffers();                          // Replaces glFlush() to use double buffering
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param key    [in] Key code.
//! \param x      [in] X-coordinate of mouse when key is pressed.
//! \param y      [in] Y-coordinate of mouse when key is pressed.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
	switch (key) {
		//|____________________________________________________________________
		//|
		//| Camera switch
		//|____________________________________________________________________

	case 'v': // Select camera to view
		cam_id = (cam_id + 1) % 3;
		printf("View camera = %d\n", cam_id);
		break;
	case 'b': // Select camera to control
		camctrl_id = (camctrl_id + 1) % 3;
		printf("Control camera = %d\n", camctrl_id);
		break;

		//|____________________________________________________________________
		//|
		//| Plane 2 controls
		//|____________________________________________________________________

	case 's': { // Forward translation of the plane (+Z translation)  
		gmtl::Quatf v_q = plane_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
		plane_p = plane_p + v_q.mData;
	} break;
	case 'f': { // Backward translation of the plane (-Z translation)
		gmtl::Quatf v_q = plane_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
		plane_p = plane_p + v_q.mData;
	} break;

	case 'e': // Rolls the plane (+Z rot)
		plane_q = plane_q * zrotp_q;
		break;
	case 'q': // Rolls the plane (-Z rot)
		plane_q = plane_q * zrotn_q;
		break;

	case 'x': // Pitches the plane (+X rot)
		plane_q = plane_q * xrotp_q;
		break;
	case 'w': // Pitches the plane (-X rot)
		plane_q = plane_q * xrotn_q;
		break;

	case 'a': // Yaws the plane (+Y rot)
		plane_q = plane_q * yrotp_q;
		break;
	case 'd': // Yaws the plane (-Y rot)
		plane_q = plane_q * yrotn_q;
		break;


		//|____________________________________________________________________
		//|
		//| Plane 1 controls
		//|____________________________________________________________________

	case 'S': { // Forward translation of the plane (+Z translation)  
		gmtl::Quatf v_q = plane_q1 * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q1);
		plane_p1 = plane_p1 + v_q.mData;
	} break;
	case 'F': { // Backward translation of the plane (-Z translation)
		gmtl::Quatf v_q = plane_q1 * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q1);
		plane_p1 = plane_p1 + v_q.mData;
	} break;

	case 'E': // Rolls the plane (+Z rot)
		plane_q1 = plane_q1 * zrotp_q;
		break;
	case 'Q': // Rolls the plane (-Z rot)
		plane_q1 = plane_q1 * zrotn_q;
		break;

	case 'X': // Pitches the plane (+X rot)
		plane_q1 = plane_q1 * xrotp_q;
		break;
	case 'W': // Pitches the plane (-X rot)
		plane_q1 = plane_q1 * xrotn_q;
		break;

	case 'A': // Yaws the plane (+Y rot)
		plane_q1 = plane_q1 * yrotp_q;
		break;
	case 'D': // Yaws the plane (-Y rot)
		plane_q1 = plane_q1 * yrotn_q;
		break;

		//|____________________________________________________________________
		//|
		//| Propeller controls (subpart)
		//|____________________________________________________________________

	case 'r': // Rotates right propeller 
		pp_angle += PROPELLER_ROTATION;
		break;
	case 't': // Rotates left propeller 
		pp_angle_left += PROPELLER_ROTATION;
		break;
	case 'y': // Rotates top propeller 
		pp_angle_top += PROPELLER_ROTATION;
		break;
	case 'u': // Rotates subsubpart propeller 
		pp_angle_ss += PROPELLER_ROTATION;
		break;

		// TODO: Add the remaining controls/transforms        
	}

	glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MouseFunc
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void MouseFunc(int button, int state, int x, int y)
{
	int km_state;

	// Updates button's sate and mouse coordinates
	if (state == GLUT_DOWN) {
		mbuttons[button] = true;
		mx_prev = x;
		my_prev = y;
	}
	else {
		mbuttons[button] = false;
	}

	// Updates keyboard modifiers
	km_state = glutGetModifiers();
	kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
	kmodifiers[KM_CTRL] = km_state & GLUT_ACTIVE_CTRL ? true : false;
	kmodifiers[KM_ALT] = km_state & GLUT_ACTIVE_ALT ? true : false;

	//glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MotionFunc
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void MotionFunc(int x, int y)
{
	int dx, dy, d;

	if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON]) {
		// Computes distances the mouse has moved
		dx = x - mx_prev;
		dy = y - my_prev;

		// Updates mouse coordinates
		mx_prev = x;
		my_prev = y;

		// Hold left button to rotate camera
		if (mbuttons[GLUT_LEFT_BUTTON]) {
			if (!kmodifiers[KM_CTRL]) {
				elevation[camctrl_id] += dy;            // Elevation update
			}
			if (!kmodifiers[KM_SHIFT]) {
				azimuth[camctrl_id] += dx;             // Azimuth update
			}
		}

		// Hold right button to zoom
		if (mbuttons[GLUT_RIGHT_BUTTON]) {
			if (abs(dx) >= abs(dy)) {
				d = dx;
			}
			else {
				d = -dy;
			}
			distance[camctrl_id] += d;
		}

		glutPostRedisplay();      // Asks GLUT to redraw the screen
	}
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
	// Track the current window dimensions
	w_width = w;
	w_height = h;
	glViewport(0, 0, (GLsizei)w_width, (GLsizei)w_height);
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
	glBegin(GL_LINES);
	// X axis is red
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(l, 0.0f, 0.0f);

	// Y axis is green
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, l, 0.0f);

	// Z axis is blue
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, l);
	glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawPlaneBody
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws a plane body.
//|____________________________________________________________________

void drawCube(const float width, const float length, const float height, const float colours[3]) {
	float w2 = width / 2;
	float h2 = height / 2;
	float l2 = length / 2;

	// modify the copy only (arrays are passed by reference)
	float colours_copy[3] = { colours[0], colours[1], colours[2] };

	// for adding shadow, increase this to add contrast, vice versa
	float c_delta = 0.05f;

	glBegin(GL_QUADS);

	// front face
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(w2, h2, -l2);
	glVertex3f(-w2, h2, -l2);
	glVertex3f(-w2, -h2, -l2);
	glVertex3f(w2, -h2, -l2);

	// increase brightness
	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// right face
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(w2, h2, -l2);
	glVertex3f(w2, h2, l2);
	glVertex3f(w2, -h2, l2);
	glVertex3f(w2, -h2, -l2);

	// increase brightness
	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// top face
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(w2, h2, l2);
	glVertex3f(-w2, h2, l2);
	glVertex3f(-w2, h2, -l2);
	glVertex3f(w2, h2, -l2);

	// increase brightness
	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// bottom face
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(w2, -h2, -l2);
	glVertex3f(-w2, -h2, -l2);
	glVertex3f(-w2, -h2, l2);
	glVertex3f(w2, -h2, l2);

	// increase brightness
	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// back face
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(-w2, h2, l2);
	glVertex3f(w2, h2, l2);
	glVertex3f(w2, -h2, l2);
	glVertex3f(-w2, -h2, l2);

	// increase brightness
	colours_copy[0] += c_delta;
	colours_copy[1] += c_delta;
	colours_copy[2] += c_delta;

	// left face
	glColor3f(colours_copy[0], colours_copy[1], colours_copy[2]);
	glVertex3f(-w2, h2, -l2);
	glVertex3f(-w2, h2, l2);
	glVertex3f(-w2, -h2, l2);
	glVertex3f(-w2, -h2, -l2);
	glEnd();
}

void DrawTurtleShell(const float width, const float length, const float height)
{
	float w = width / 2;
	float l = length / 2;

	glBegin(GL_QUADS);
	drawCube(P_WIDTH, P_LENGTH, P_HEIGHT, colour_brown);
	//// Body is red
	//glColor3f( 1.0f, 0.0f, 0.0f);
	//glVertex3f(0.0f, 0.0f,   l);
	//  glVertex3f(   w, 0.0f,  -l);
	//  glVertex3f(  -w, 0.0f,  -l);

	//// Wing is blue
	//glColor3f( 0.0f,    0.0f, 1.0f);
	//glVertex3f(0.0f,    0.0f, 0.0f);
	//  glVertex3f(0.0f,    0.0f,   -l);
	//  glVertex3f(0.0f,  height,   -l);
	glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawPropeller
//|
//! \param width       [in] Width  of the propeller.
//! \param length      [in] Length of the propeller.
//! \return None.
//!
//! Draws a propeller.
//|____________________________________________________________________

void DrawPropeller(const float width, const float length)
{
	float w = width / 2;
	float l = length / 2;

	glBegin(GL_QUADS);
	// Propeller is white
	glColor3f(1.0f, 1.0f, 1.0f);
	glVertex3f(-w, -l, 0.0f);
	glVertex3f(w, -l, 0.0f);
	glVertex3f(w, l, 0.0f);
	glVertex3f(-w, l, 0.0f);
	glEnd();
}

//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char** argv)
{
	InitTransforms();

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);     // Uses GLUT_DOUBLE to enable double buffering
	glutInitWindowSize(w_width, w_height);

	glutCreateWindow("Plane Episode 2");

	glutDisplayFunc(DisplayFunc);
	glutKeyboardFunc(KeyboardFunc);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MotionFunc);
	glutReshapeFunc(ReshapeFunc);

	InitGL();

	glutMainLoop();

	return 0;
}