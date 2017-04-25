#version 330 core

in vec2 TextCoord;
in vec3 vNormal;
uniform sampler2D texture_diffuse0;
uniform sampler2D texture_diffuse1;
uniform sampler2D texture_diffuse2;
uniform sampler2D texture_specular0;
uniform sampler2D texture_specular1;
uniform sampler2D texture_specular2;

out vec4 color;


void main()
{
	//color = texture(texture_diffuse0, TextCoord);
	//color = vec4(1f,1f,1f,1f);

	//利用法向量描边
	
   float silhouette = length(vNormal * vec3(0.0, 0.0, 1.0));
    if (silhouette < 0.1) {
        silhouette = 0.0;
    }
    else {
        silhouette = 1.0;
    }

    gl_FragColor = vec4(silhouette, silhouette, silhouette, 1.0);

}