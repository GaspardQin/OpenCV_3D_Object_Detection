// ����GLEW�� ���徲̬����
//#define GLEW_STATIC
#include "loadModel.h"
GLfloat deltaTime = 0.0f; // ��ǰ֡����һ֡��ʱ���
GLfloat lastFrame = 0.0f; // ��һ֡ʱ��
GLfloat camera_z = 0.0f;
GLfloat pos_model_set[3] = { 0.0f, 0.0f, -100.0f };
Camera camera = Camera(glm::vec3(0.0f, 0.0f, 0.0f));
GLfloat rotate_degree_set[3] = { 0.0f };
bool firstMouseMove = true;
GLfloat lastX = WINDOW_WIDTH / 2.0f, lastY = WINDOW_HEIGHT / 2.0f;
//glm::vec3 vec_scale = glm::vec3(1.f, 1.f, 1.f);
glm::mat4 projection;
glm::mat4 view;
glm::mat4 M_model;
DWORD WINAPI glThreadFun(LPVOID lpParmeter)
{
	
	if (!glfwInit())	// ��ʼ��glfw��
	{
		std::cout << "Error::GLFW could not initialize GLFW!" << std::endl;
		return 0;
	}

	// ����OpenGL 3.3 core profile
	std::cout << "Start OpenGL core profile version 3.3" << std::endl;
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_SAMPLES, 4);
	// ��������
	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,
		"Loading model with AssImp", NULL, NULL);
	if (!window)
	{
		std::cout << "Error::GLFW could not create winddow!" << std::endl;
		glfwTerminate();
		std::system("pause");
		return 0;
	}
	// �����Ĵ��ڵ�contextָ��Ϊ��ǰcontext
	glfwMakeContextCurrent(window);

	// ע�ᴰ�ڼ����¼��ص�����
	glfwSetKeyCallback(window, key_callback);
	// ע������¼��ص�����
	//glfwSetCursorPosCallback(window, mouse_move_callback);
	// ע���������¼��ص�����
	glfwSetScrollCallback(window, mouse_scroll_callback);
	// ��겶�� ͣ���ڳ�����
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// ��ʼ��GLEW ��ȡOpenGL����
	glewExperimental = GL_TRUE; // ��glew��ȡ������չ����
	GLenum status = glewInit();
	if (status != GLEW_OK)
	{
		std::cout << "Error::GLEW glew version:" << glewGetString(GLEW_VERSION) 
			<< " error string:" << glewGetErrorString(status) << std::endl;
		glfwTerminate();
		std::system("pause");
		return 0;
	}

	// �����ӿڲ���
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	
	//Section1 ����ģ������ Ϊ�˷������ģ�� ���Ǵ��ļ���ȡģ���ļ�·��
	Model objModel;
	std::ifstream modelPath("modelPath.txt");
	if (!modelPath)
	{
		std::cerr << "Error::could not read model path file." << std::endl;
		glfwTerminate();
		std::system("pause");
		return 0;
	}
	std::string modelFilePath;
	std::getline(modelPath, modelFilePath);
	if (modelFilePath.empty())
	{
		std::cerr << "Error::model path empty." << std::endl;
		glfwTerminate();
		std::system("pause");
		return 0;
	}
	if (!objModel.loadModel(modelFilePath))
	{
		glfwTerminate();
		std::system("pause");
		return 0;
	}
	// Section2 ׼����ɫ������
	Shader shader("model.vertex", "model.frag");

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);
	//glEnable(GL_LINE_SMOOTH);
	
	glEnable(GL_CULL_FACE);
	//glStencilFunc(GL_NOTEQUAL, 1, -1);
	//glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	//glLineWidth(3);
	//glPolygonMode(GL_FRONT, GL_LINE);
	// ��ʼ��Ϸ��ѭ��

	while (!glfwWindowShouldClose(window))
	{
//		WaitForSingleObject(readImgEvent,INFINITE);
		WaitForSingleObject(readModelEvent, INFINITE);

		GLfloat currentFrame = (GLfloat)glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		//if (deltaTime < 1/20) continue;
		lastFrame = currentFrame;
		glfwPollEvents(); // ����������� ���̵��¼�
		//do_movement(); // �����û�������� �����������
		print_model_info();//print the model info;
		// �����ɫ������ ����Ϊָ����ɫ
		glClearColor(0.f, 0.0f, 0.f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		shader.use();
		
		projection = glm::perspective(camera.mouse_zoom,
			(GLfloat)(WINDOW_WIDTH) / WINDOW_HEIGHT, 10.0f, 1000.0f); // ͶӰ����
		view = camera.getViewMatrix(); // �ӱ任����
		glUniformMatrix4fv(glGetUniformLocation(shader.programId, "projection"),
			1, GL_FALSE, glm::value_ptr(projection));
		glUniformMatrix4fv(glGetUniformLocation(shader.programId, "view"),
			1, GL_FALSE, glm::value_ptr(view));
		glm::mat4 model = glm::mat4(1.0);
		model = glm::translate(model, glm::vec3(pos_model_set[0], pos_model_set[1], pos_model_set[2])); // �ٵ���λ��
		rotate_model(rotate_degree_set, model); //����ת
		M_model = model;
		//model = glm::scale(model,vec_scale); // �ʵ���Сģ��
		glUniformMatrix4fv(glGetUniformLocation(shader.programId, "model"),
			1, GL_FALSE, glm::value_ptr(model));
		// ������д�������ƴ���
		
		objModel.draw(shader); // ��������
		
		glBindVertexArray(0);
		
		glUseProgram(0);
		//display_axis();

		//read pixels into opencv mat

		glReadPixels(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, readSrcImg.data);
		cv::flip(readSrcImg, readSrcImg, 0);
		glfwSwapBuffers(window); // ��������




		ResetEvent(readModelEvent);
		SetEvent(sentModelEvent);
		
		
	}
	// �ͷ���Դ
	glfwTerminate();
	return 0;
}
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	switch (key)
	{
	case GLFW_KEY_W:camera.handleKeyPress(FORWARD, deltaTime);
		break;
	case GLFW_KEY_S:camera.handleKeyPress(BACKWARD, deltaTime);
		break;
	case GLFW_KEY_A:camera.handleKeyPress(LEFT, deltaTime);
		break;
	case GLFW_KEY_D:camera.handleKeyPress(RIGHT, deltaTime);
		break;
	case GLFW_KEY_Q:camera.handleKeyPress(UP, deltaTime);
		break;
	case GLFW_KEY_Z:camera.handleKeyPress(DOWN, deltaTime);
		break;
	case GLFW_KEY_UP: add_rotate_degree(0, 1, 0);
		break;
	case GLFW_KEY_LEFT: add_rotate_degree(1, 0, 0);
		break;
	case GLFW_KEY_RIGHT: add_rotate_degree(-1, 0, 0);
		break;
	case GLFW_KEY_DOWN: add_rotate_degree(0, -1, 0);
		break;
	case GLFW_KEY_LEFT_BRACKET: add_rotate_degree(0, 0, 1);
		break;
	case GLFW_KEY_RIGHT_BRACKET: add_rotate_degree(0, 0, -1);
		break;

	default:
		break;
	}
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, GL_TRUE); // �رմ���
	}
}
void mouse_scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.handleMouseScroll(yoffset);
}
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouseMove) // �״�����ƶ�
	{
		lastX = xpos;
		lastY = ypos;
		firstMouseMove = false;
	}

	GLfloat xoffset = xpos - lastX;
	GLfloat yoffset = lastY - ypos;

	lastX = xpos;
	lastY = ypos;

	camera.handleMouseMove(xoffset, yoffset);
}
void rotate_model(GLfloat rotate_degree_set[], glm::mat4& mat_rotate) {
	glm::mat4 only_rotate;
	only_rotate = glm::eulerAngleYXZ(glm::radians(rotate_degree_set[1]), glm::radians(rotate_degree_set[0]), glm::radians(rotate_degree_set[2]));//yawPitchRoll˳��
	mat_rotate = mat_rotate*only_rotate;
																																				 //mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[2]), glm::vec3(0.0, 0.0, 1.0));//���վ���˷����ȳ˵ģ�������ߵģ�����Ч
	//mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[1]), glm::vec3(0.0, 1.0, 0.0));
	//mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[0]), glm::vec3(1.0, 0.0, 0.0));


}
void set_rotate_degree(GLfloat x_degree, GLfloat y_degree, GLfloat z_degree) {
	rotate_degree_set[0] = x_degree;
	rotate_degree_set[1] = y_degree;
	rotate_degree_set[2] = z_degree;
}
GLfloat* get_rotate_degree() {
	GLfloat* rd = rotate_degree_set;
	return rd;
}
void print_rotate_degree() {
	std::cout << "x_degree  " << rotate_degree_set[0] << "  y_degree  " << rotate_degree_set[1] << "  z_degree  " << rotate_degree_set[2] << std::endl;
}
void add_rotate_degree(GLfloat x_add, GLfloat y_add, GLfloat z_add) {
	rotate_degree_set[0] = rotate_degree_set[0] +x_add;
	rotate_degree_set[1] = rotate_degree_set[1] +y_add;
	rotate_degree_set[2] = rotate_degree_set[2] +z_add;
	
}
void print_model_info() {
	std::cout << "model's info : " << std::endl;
	std::cout << "	pos: x " << pos_model_set[0] << "  y  " << pos_model_set[1] << "  z  " << pos_model_set[2] << std::endl;
	std::cout << "	rotate: x" << rotate_degree_set[0] << "  y  " << rotate_degree_set[1] << " z  " << rotate_degree_set[2] << std::endl;


}
void display_axis() {
	GLuint displayList = glGenLists(1);  //������ʾ�б�����

	glNewList(displayList, GL_COMPILE);   //������ʾ�б�

	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(10, 0, 0);
	glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 10, 0);
	glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 10);
	glEnd();

	glEndList();
	glCallList(displayList);
}