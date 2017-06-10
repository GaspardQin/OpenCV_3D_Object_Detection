// ����GLEW�� ���徲̬����
//#define GLEW_STATIC
#include "stdafx.h"
#include "loadModel.h"
namespace Object_Detection {
	GLfloat deltaTime = 0.0f; // ��ǰ֡����һ֡��ʱ���
	GLfloat lastFrame = 0.0f; // ��һ֡ʱ��
	GLfloat camera_z = 0.0f;
	GLfloat pos_model_set[3] = { 0.0f, 0.0f, -100.0f };
	Camera camera = Camera(glm::vec3(0.0f, 0.0f, 0.0f));
	float rotate_degree_set[3] = { 0.0f };
	glm::quat quat_set = glm::quat(glm::vec3(0, 0, 0));
	bool firstMouseMove = true;
	GLfloat lastX = WINDOW_WIDTH / 2.0f, lastY = WINDOW_HEIGHT / 2.0f;
	//glm::vec3 vec_scale = glm::vec3(1.f, 1.f, 1.f);
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 M_model;
	void glThread::getGLROIrect(double x, double y, double z, int* lower_left_corner,int& modified_window_width, int& modified_window_height) {
		//ȷ��OpenGL���½�λ�ã���Ӧopencv���Ͻǣ�
		int column_left_pixel = round(WINDOW_WIDTH / 2 + x / (CCD_WIDTH / 2 / FOCAL_DISTANCE*(-z))*WINDOW_WIDTH - ROI_WIDTH / 2);//zΪ����
		int row_top_pixel = round(WINDOW_HEIGHT / 2 - y / (CCD_WIDTH / 2 / FOCAL_DISTANCE*(-z) / WINDOW_WIDTH*WINDOW_HEIGHT)*WINDOW_HEIGHT - ROI_HEIGHT / 2);//y���µߵ�
		int column_right_pixel = column_left_pixel + ROI_WIDTH;
		int row_bottom_pixel = row_top_pixel + ROI_HEIGHT;
		//��ֹ�����߽�
		if (column_left_pixel < 0) column_left_pixel = 0;
		if (row_top_pixel < 0) row_top_pixel = 0;
		if (column_right_pixel >= WINDOW_WIDTH) column_right_pixel = WINDOW_WIDTH - 1;
		if (row_bottom_pixel >= WINDOW_HEIGHT) row_bottom_pixel = WINDOW_HEIGHT - 1;
		lower_left_corner[0] = column_left_pixel;
		lower_left_corner[1] = row_top_pixel; //OpenGL�����½Ƕ�ӦOpenCV�����Ͻ�
		modified_window_height = row_bottom_pixel - row_top_pixel;
		modified_window_width = column_right_pixel - column_left_pixel;
		//lower_left_corner[2] = column_right_pixel;
		//lower_left_corner[3] = row_bottom_pixel;
	}
	void glThread::glThreadFun()
	{

		if (!glfwInit())	// ��ʼ��glfw��
		{
			std::cout << "Error::GLFW could not initialize GLFW!" << std::endl;
			return;
		}

		// ����OpenGL 3.3 core profile
		std::cout << "Start OpenGL core profile version 3.3" << std::endl;
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

		// ��������
		GLFWwindow* window = glfwCreateWindow(1, 1,
			"Loading model with AssImp", NULL, NULL);
		glfwHideWindow(window);
		if (!window)
		{
			std::cout << "Error::GLFW could not create winddow!" << std::endl;
			glfwTerminate();
			std::system("pause");
			return;
		}
		// �����Ĵ��ڵ�contextָ��Ϊ��ǰcontext
		glfwMakeContextCurrent(window);


		// ��ʼ��GLEW ��ȡOpenGL����
		glewExperimental = GL_TRUE; // ��glew��ȡ������չ����
		GLenum status = glewInit();
		if (status != GLEW_OK)
		{
			std::cout << "Error::GLEW glew version:" << glewGetString(GLEW_VERSION)
				<< " error string:" << glewGetErrorString(status) << std::endl;
			glfwTerminate();
			std::system("pause");
			return;
		}



		//Somewhere at initialization
		GLuint fbo, fbo_render, fbo_texture; //fbo, 
		glGenFramebuffers(1, &fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		//std::cout << glGetError() << std::endl;

		glGenTextures(1, &fbo_texture);
		glBindTexture(GL_TEXTURE_2D, fbo_texture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WINDOW_WIDTH, WINDOW_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL); //ĩβ��NULL��ʾ����ֻԤ����ռ䣬����ʵ�ʼ�������
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture, 0);



		//glGenRenderbuffers(1, &fbo_render);
		//std::cout << glGetError() << std::endl;
		//glBindRenderbuffer(GL_RENDERBUFFER, fbo_render);
		//std::cout << glGetError() << std::endl;
		//glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, WINDOW_WIDTH, WINDOW_HEIGHT);
		//std::cout << glGetError() << std::endl;
		//glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, fbo_render);
		//std::cout << glGetError() << std::endl;
		//glBindRenderbuffer(GL_RENDERBUFFER, 0);
		//std::cout << glGetError() << std::endl;
		GLuint fbo_depth;
		glGenRenderbuffers(1, &fbo_depth);
		glBindRenderbuffer(GL_RENDERBUFFER, fbo_depth);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, WINDOW_WIDTH, WINDOW_HEIGHT);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, fbo_depth);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);
		//Before drawing
		//glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
		//std::cout << glGetError() << std::endl;


		// �����ӿڲ���
		glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
		//std::cout << glGetError() << std::endl;
		//Section1 ����ģ������ Ϊ�˷������ģ�� ���Ǵ��ļ���ȡģ���ļ�·��
		Model objModel;
		std::ifstream modelPath("../src/modelPath.txt");
		if (!modelPath)
		{
			std::cerr << "Error::could not read model path file." << std::endl;
			glfwTerminate();
			std::system("pause");
			return;
		}
		std::string modelFilePath;
		std::getline(modelPath, modelFilePath);
		if (modelFilePath.empty())
		{
			std::cerr << "Error::model path empty." << std::endl;
			glfwTerminate();
			std::system("pause");
			return;
		}
		if (!objModel.loadModel(modelFilePath))
		{
			glfwTerminate();
			std::system("pause");
			return;
		}
		// Section2 ׼����ɫ������
		Shader shader_silhouette("../src/model.vertex", "../src/model.frag", "../src/model.geometry");
		Shader shader_object("../src/model.vertex", "../src/model_object.frag", "../src/model_object.geometry"); //Ϊ�����ص������ص����߿�
		//Shader shader("../src/model.vertex", "../src/model.geometry");
		glEnable(GL_DEPTH_TEST);

		//glEnable(GL_MULTISAMPLE);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glEnable(GL_POLYGON_OFFSET_LINE);
		glEnable(GL_POLYGON_OFFSET_POINT);
		glEnable(GL_CULL_FACE);

		projection = glm::perspective(GLfloat(glm::atan(CCD_WIDTH / 2.0 / FOCAL_DISTANCE)),
			(GLfloat)(WINDOW_WIDTH / WINDOW_HEIGHT), 20.0f, 1000.0f); // ͶӰ����
		view = camera.getViewMatrix(); // �ӱ任����

		//******************* 
		//��OpenGL��ȡͼƬ��ʱ������µߵ����������Ⱦ��ʱ�򼴵ߵ�ͼƬ�ɱ���ʹ��cv::flip()�������ú�����������
		projection = glm::scale(projection, glm::vec3(1.0, -1.0, 1.0));//ͶӰ�������µߵ�
		glFrontFace(GL_CW); //ͶӰ�������µߵ�����Ҫ����ǰ��������жϷ���
		//*******************


		glm::mat4 model;
		glDepthFunc(GL_LEQUAL);
		while (!glfwWindowShouldClose(window))
		{


			WaitForSingleObject(nextModelEvent, INFINITE);

			//GLfloat currentFrame = (GLfloat)glfwGetTime();
		//	deltaTime = currentFrame - lastFrame;
			//if (deltaTime < 1/20) continue;
		//	lastFrame = currentFrame;
			glfwPollEvents(); // ����������� ���̵��¼�
			if (print_option == DETAIL_PRINT)  print_model_info();//print the model info;
			// �����ɫ������ ����Ϊָ����ɫ
			glClearColor(0.f, 0.0f, 0.f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			model = glm::mat4(1.0);
			model = glm::translate(model, glm::vec3(pos_model_set[0], pos_model_set[1], pos_model_set[2])); // �ٵ���λ��
			rotate_model(rotate_degree_set, model); //����ת
													//	M_model = model;
													//model = glm::scale(model,vec_scale); // �ʵ���Сģ��
			std::cout << glGetError() << std::endl;



			glPolygonOffset(1.f, 0.f); //���z-fighting ���⣬object���ֻ��Ƶ�ʱ�����һ��������ƫ��



			shader_object.use();


			glUniformMatrix4fv(glGetUniformLocation(shader_object.programId, "projection"),
				1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(glGetUniformLocation(shader_object.programId, "view"),
				1, GL_FALSE, glm::value_ptr(view));
			glUniformMatrix4fv(glGetUniformLocation(shader_object.programId, "model"),
				1, GL_FALSE, glm::value_ptr(model));
			objModel.offscreenDraw(shader_object, fbo); // ��������
			//std::cout << glGetError() << std::endl;

			glPolygonOffset(0.0f, 0.f); //���ڱ߿���Ʋ��֣���ƫ������Ϊ0
			shader_silhouette.use();

			glUniformMatrix4fv(glGetUniformLocation(shader_silhouette.programId, "projection"),
				1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(glGetUniformLocation(shader_silhouette.programId, "view"),
				1, GL_FALSE, glm::value_ptr(view));
			glUniformMatrix4fv(glGetUniformLocation(shader_silhouette.programId, "model"),
				1, GL_FALSE, glm::value_ptr(model));
			//std::cout << glGetError() << std::endl;
			objModel.offscreenDraw(shader_silhouette, fbo); // ��������



			glUseProgram(0);

			glBindFramebuffer(GL_FRAMEBUFFER, fbo);

			glReadBuffer(GL_COLOR_ATTACHMENT0);//ָ��glReadPixels��ȡGL_COLOR_ATTACHMENT0����

			WaitForSingleObject(readImgEvent, 50);//��һ����Ⱦͼ����Ҫ��ȡ���

			if (option == MODEL_DT_CAM_CANNY_ONLINE_ROI) {
				//ֱ�Ӷ�ȡROI����
				int lower_left_corner[2]; int modified_window_width; int modified_window_height;
				getGLROIrect(pos_model_set[0], pos_model_set[1], pos_model_set[2], lower_left_corner, modified_window_width,modified_window_height);
				
				//debug use
				if(is_create_sample == true) glReadPixels(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_GREEN, GL_UNSIGNED_BYTE, readSrcImg.data); //�������϶�ȡ�������Ҫ��ת
			
				glReadPixels(lower_left_corner[0], lower_left_corner[1], modified_window_width, modified_window_height, GL_GREEN, GL_UNSIGNED_BYTE, readSrcImgROI.data); //�������϶�ȡ�������Ҫ��ת

			}
			else glReadPixels(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_GREEN, GL_UNSIGNED_BYTE, readSrcImg.data); //�������϶�ȡ�������Ҫ��ת
			//��Ҫ�����������Դ����ڴ��еĴ������ݣ�������PCIE���ٶ�
			//���ʹ�ù����ڴ��intel �����Կ� �����������ʹ˲�������

			//cv::flip(readSrcImg, readSrcImg, 0); //ʱ�����Ĺ��ߣ����ø���projection �������
			//std::cout << glGetError() << std::endl;
			if (glGetError() != 0) {
				while (1) {
					std::cout << "OpenGL Error" << std::endl;
				}
			}


			ResetEvent(nextModelEvent);
			SetEvent(sentModelEvent);


		}
		// �ͷ���Դ

		glDeleteRenderbuffers(1, &fbo);


		glfwTerminate();
		return;
	}


	void glThread::rotate_model(float rotate_degree_set[], glm::mat4& mat_rotate) {
		glm::mat4 only_rotate;
		only_rotate = glm::eulerAngleYXZ(glm::radians(rotate_degree_set[1]), glm::radians(rotate_degree_set[0]), glm::radians(rotate_degree_set[2]));//yawPitchRoll˳��
		mat_rotate = mat_rotate*only_rotate;
		//mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[2]), glm::vec3(0.0, 0.0, 1.0));//���վ���˷����ȳ˵ģ�������ߵģ�����Ч
		//mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[1]), glm::vec3(0.0, 1.0, 0.0));
		//mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[0]), glm::vec3(1.0, 0.0, 0.0));


	}

	void glThread::print_rotate_degree() {
		std::cout << "x_degree  " << rotate_degree_set[0] << "  y_degree  " << rotate_degree_set[1] << "  z_degree  " << rotate_degree_set[2] << std::endl;
	}
	void glThread::print_model_info() {
		//	glm::vec3 debug_angle = glm::degrees(glm::eulerAngles(quat_set));

		std::cout << "model's info : " << std::endl;
		std::cout << "	pos: x " << pos_model_set[0] << "  y  " << pos_model_set[1] << "  z  " << pos_model_set[2] << std::endl;
		std::cout << "	rotate: x" << rotate_degree_set[0] << "  y  " << rotate_degree_set[1] << " z  " << rotate_degree_set[2] << std::endl;


	}
}