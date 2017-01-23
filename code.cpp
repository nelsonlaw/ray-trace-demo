#include "stdafx.h"
#include "code.h"

int winWidth = 640;		// window width
int winHeight = 480;	// window height


V3 ViewPoint;		// view point
V3 ImageLL;		// coordinates at lower left corner of image
V3 ImageLR;		// coordinates at lower right corner of image
V3 ImageUL;		// coordinates at upper left corner of image
V3 ImageUR;		// coordinates at upper right corner of image

int MaxTraceDepth = 5;			// depth of recursive ray-tracing

// scene objects
vector<CLightSource *> vLightSource;		// array of light sources
vector<CPrimitive *> vObjects;				// array of objects


void RayTracing(V3 * colorMap)
{	

	//add your code here
	V3 rayStart = ViewPoint;
	for(int i=0;i<winWidth;i++){
		for (int j=0;j<winHeight;j++){
			V3 worldCo = ImageLL+i*(ImageLR-ImageLL)/(winWidth-1)+j*(ImageUL-ImageLL)/(winHeight-1);
			V3 rayDir = (worldCo - rayStart);
			rayDir.normalize();
			V3 color;
			Trace(rayStart,rayDir,0,color);
			colorMap[j*winWidth+i][0]=color[0];
			colorMap[j*winWidth+i][1]=color[1];
			colorMap[j*winWidth+i][2]=color[2];
		}
	}
}

void Trace(V3& rayStart, V3& rayDir, int depth, V3& color)
{
	//add your code here
	CPrimitive* objHit;
	V3 intersection;
	V3 normal;
	if(Intersect(rayStart, rayDir, objHit, intersection, normal)){
		Shade(objHit,rayStart, rayDir, intersection, normal, depth, color);
		//color[0]=color[1]=color[2]=1;
	}else{
		color[0]=color[1]=color[2]=0;
	}
}

// compute color at a given point
void Shade(CPrimitive *obj,V3& rayStart, V3& rayDir, V3& intersection, V3& normal, int depth, V3& color)
{
	//add your code here
	
	V3 DS;
	DS[0] = DS[1] = DS[2] = 0;

	V3 Oa;
	obj->GetAmbient(intersection, Oa);
	V3 Od;
	obj->GetDiffuse(intersection,Od);
	V3 Os;
	obj->GetSpecular(intersection,Os);

	float Kt = 1-obj->m_Opacity;
	float n = obj->m_Shininess;
	float Ks = obj->m_Reflectance;

	V3 V = rayStart - intersection;
	V.normalize();
	
	color[0] = Oa[0];
	color[1] = Oa[1];
	color[2] = Oa[2];

	for (int i = 0; i < (int)vLightSource.size(); i++) {
		V3 Li;
		V3 lightPos = vLightSource[i]->position;
		Li = lightPos - intersection;
		Li.normalize();
		V3 sRay = (-Li);
		sRay.normalize();

		if(normal.dot(Li)>0){
			V3 intersectionTemp;
			V3 normalTemp;	
			int Si;
			Si = Intersect(lightPos, sRay , obj, intersectionTemp, normalTemp);
			if(Si){
				V3 Ipi;
				Ipi = vLightSource[i]->color;
				V3 R = 2 * normal.dot(Li) * normal - Li;
				R.normalize();
				V3 temp = ( (1-Kt) * Od * normal.dot(Li) + Ks * Os * pow(R.dot(V),n) );
				temp[0]=Ipi[0]* temp[0];
				temp[1]=Ipi[1]* temp[1];
				temp[2]=Ipi[2]* temp[2];
				DS += temp;
			}
		}
		
	}
	color += DS;
	
	if(depth < MaxTraceDepth){
		if(Ks>0){
			V3 rRay = (-V) - 2 * normal.dot(-V) * normal;
			rRay.normalize();
			V3 rColor;
			Trace( intersection, rRay, depth + 1,rColor );
			rColor *= Ks;
			color += rColor;
		}
	}
	
	if (color[0]>1 ) color[0] = 1;
	if (color[1]>1 ) color[1] = 1;
	if (color[2]>1 ) color[2] = 1;

}

bool IntersectQuadratic(V3 rayStart,V3 rayDir, float* coeffMatrix,float& t, V3& intersection)
{		
	//add your code here
	float S[4] = {rayStart[0],rayStart[1],rayStart[2],1};
	float D[4] = {rayDir[0],rayDir[1],rayDir[2],0};
	float temp[4];


	VectorMultMatrix(D,coeffMatrix,temp);
	float a = VectorMultVector(temp,D);

	VectorMultMatrix(S,coeffMatrix,temp);
	float b = VectorMultVector(temp,D);
	b *= 2;

	VectorMultMatrix(S,coeffMatrix,temp);
	float c = VectorMultVector(temp,S);

	float delta = b*b - 4*a*c;
	if(delta>0){
		float t0,t1;
		t0 = (-b+sqrt(delta))/(2*a);
		t1 = (-b-sqrt(delta))/(2*a);
		if(t0 < t1 && t0 > 0) t = t0;
		else if (t1 < t0 && t1 > 0) t = t1;
		else return false;

	}else if(delta == 0){
		t = -b/(2*a);
		if(t<0) return false;
	}else return false;

	intersection = rayStart + t*rayDir;
	return true;
	
}
bool  IntersectTriangle(V3 rayStart,V3 rayDir, V3 v0, V3 v1,V3 v2, float& t,V3& intersection)
{	
	//add your code here
	V3 N = (v1-v0).cross(v2-v0);
	if(N.dot(rayDir)!=0){
		t = -(rayStart.dot(N)+(-v0.dot(N)))/(rayDir.dot(N));
	}else{
		return false;
	}
	intersection = rayStart + t*rayDir;
	V3 cp0 = v0 - intersection;
	V3 cp1 = v1 - intersection;
	V3 cp2 = v2 - intersection;
	V3 cross01 = cp0.cross(cp1); 
	V3 cross12 = cp1.cross(cp2); 
	V3 cross20 = cp2.cross(cp0); 
	if(cross01.dot(cross12) > 0 && cross12.dot(cross20) > 0 && cross01.dot(cross20) > 0) {
		return true;
	}
	else return false;
}

void MatrixMultVector(float *m,float *v,float *rv)//rv=m*v
{
	rv[0]=m[0]*v[0]+m[4]*v[1]+m[8]*v[2]+m[12]*v[3];
	rv[1]=m[1]*v[0]+m[5]*v[1]+m[9]*v[2]+m[13]*v[3];
	rv[2]=m[2]*v[0]+m[6]*v[1]+m[10]*v[2]+m[14]*v[3];
	rv[3]=m[3]*v[0]+m[7]*v[1]+m[11]*v[2]+m[15]*v[3];
}
void VectorMultMatrix(float *v,float *m,float *lv)//lv=v^Tm
{
	lv[0]=m[0]*v[0]+m[1]*v[1]+m[2]*v[2]+m[3]*v[3];
	lv[1]=m[4]*v[0]+m[5]*v[1]+m[6]*v[2]+m[7]*v[3];
	lv[2]=m[8]*v[0]+m[9]*v[1]+m[10]*v[2]+m[11]*v[3];
	lv[3]=m[12]*v[0]+m[13]*v[1]+m[14]*v[2]+m[15]*v[3];
}
float VectorMultVector(float *v1,float *v2)//v3=v1^Tv2
{
	return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]+v1[3]*v2[3];
}



