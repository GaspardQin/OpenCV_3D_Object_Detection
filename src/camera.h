#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <iostream>
#include <fstream>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iomanip>      // std::setprecision
// �����ƶ�����
enum Camera_Movement {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	UP,
	DOWN,
};
// ����Ԥ�賣��
const GLfloat YAW = 0.0f;
const GLfloat PITCH = 0.0f;
const GLfloat ROLL = 0.0f;
const GLfloat SPEED = 10.0f;
const GLfloat MOUSE_SENSITIVTY = 0.05f;
const GLfloat MOUSE_ZOOM = 26.99f;
const float  MAX_PITCH_ANGLE = 89.9f; // ��ֹ������

class Camera
{
public:
	Camera(glm::vec3 pos = glm::vec3(0.0, 0.0, 0.0),
		glm::vec3 up = glm::vec3(0.0, 1.0, 0.0),
		GLfloat yaw = YAW, GLfloat pitch = PITCH, GLfloat roll = ROLL) 
		:position(pos), forward(0.0, 0.0, -1.0), viewUp(up),
		moveSpeed(SPEED), mouse_zoom(MOUSE_ZOOM), mouse_sensitivity(MOUSE_SENSITIVTY),
		yawAngle(yaw), pitchAngle(pitch),rollAngle(roll)
	{
		this->updateCameraVectors();
	}
public:
	// ��ȡ�ӱ任����
	glm::mat4 getViewMatrix()
	{
		position = glm::vec3(0.0, 0.0, 0.0);
		return glm::lookAt(this->position, this->position + this->forward, this->viewUp);
	}
	glm::mat4 getViewMatrix(GLfloat* pos_model_set)
	{
		position[0] = -pos_model_set[0];
		position[1] = -pos_model_set[1];
		position[2] = -pos_model_set[2];
		return glm::lookAt(this->position, this->position + this->forward, this->viewUp);
	}
	// ������̰��������ƶ�
	void handleKeyPress(Camera_Movement direction, GLfloat deltaTime)
	{
		GLfloat velocity = this->moveSpeed * deltaTime;
		switch (direction)
		{
		case FORWARD:
			this->position += this->forward * velocity;
			break;
		case BACKWARD:
			this->position -= this->forward * velocity;
			break;
		case LEFT:
			this->position -= this->side * velocity;
			break;
		case RIGHT:
			this->position += this->side * velocity;
			break;
		case UP:
			this->position += glm::vec3(0.0, 1.0, 0.0)*velocity;
			break;
		case DOWN:
			this->position -=glm::vec3(0.0, 1.0, 0.0)*velocity;
			break;
		default:
			break;
		}
	}
	// ��������ƶ�
	void handleMouseMove(GLfloat xoffset, GLfloat yoffset)
	{
		
		xoffset *= this->mouse_sensitivity; // ����������ȵ��ڽǶȱ任
		yoffset *= this->mouse_sensitivity;

		this->pitchAngle += yoffset;
		this->yawAngle += xoffset;

		this->normalizeAngle();
		this->updateCameraVectors();
	}
	// �������������� ������[1.0, MOUSE_ZOOM]֮��
	void handleMouseScroll(GLfloat yoffset)
	{
		if (this->mouse_zoom >= 0.5f && this->mouse_zoom <= MOUSE_ZOOM)
			this->mouse_zoom -= this->mouse_sensitivity * yoffset;
		if (this->mouse_zoom <= 0.5f)
			this->mouse_zoom = 0.5f;
		if (this->mouse_zoom >= 45.0f)
			this->mouse_zoom = 45.0f;
	}
	// ʹpitch yaw�Ƕȱ����ں���Χ��
	void normalizeAngle()
	{
		if (this->pitchAngle > MAX_PITCH_ANGLE)
			this->pitchAngle = MAX_PITCH_ANGLE;
		if (this->pitchAngle < -MAX_PITCH_ANGLE)
			this->pitchAngle = -MAX_PITCH_ANGLE;
		if (this->yawAngle < 0.0f)
			this->yawAngle += 360.0f;
	}
	// ����forward side����
	void updateCameraVectors()
	{
		glm::vec3 forward;
		forward.x = -sin(glm::radians(this->yawAngle)) * cos(glm::radians(this->pitchAngle));
		forward.y = sin(glm::radians(this->pitchAngle));
		forward.z = -cos(glm::radians(this->yawAngle)) * cos(glm::radians(this->pitchAngle));
		this->forward = glm::normalize(forward);
		
		glm::vec3 side;
		side.x = cos(glm::radians(this->yawAngle));
		side.y = 0;
		side.z = -sin(glm::radians(this->yawAngle));
		this->side = glm::normalize(side);
	}
public:
	glm::vec3 forward,up, side, viewUp, position; // �������

	GLfloat yawAngle, pitchAngle,rollAngle; // ŷ����
	GLfloat moveSpeed, mouse_sensitivity, mouse_zoom; // ���ѡ��
};

#endif