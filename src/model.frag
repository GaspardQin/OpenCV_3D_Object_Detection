#version 330 core

in vec2 TextCoord;
in vec3 vNormal;
in vec3 Position;
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
	
   float silhouette = length(vNormal * vec3(0.0, 0.0, 1.0))*0.7 + 0.3;


  // vec2 xy;
   //xy[0] = Position[0];
   //xy[1] = Position[2];
   //float silhouette = vNormal[1]*0.5 + 0.3*Position[2]/20 + sin(length(xy)/3)*0.05 ;
   //color[0] = vNormal[1]*0.5;
   //color[1] = 0.3;
   //color[2] = sin(length(xy)/3)*0.2;
   //color[3] = 1;
 
  //  if (silhouette < 0.2) {
  //      silhouette = 0.2;
  //  }
	//else if (silhouette < 0.4) silhouette = 0.4;
  //  else {
 //       silhouette = 1.0;
 //   }

    color = vec4(silhouette, silhouette, silhouette, 1.0);
	//color = vec4(vNormal[0]/50+0.5, vNormal[1]/10+Position[2]/100, 0.5+vNormal[2]/5, 1.0);

}