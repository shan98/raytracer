

#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include "util.h"

Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
	SceneDagNode *childPtr;
	//std::cout << "Traversing " << node << " With child "<< node->child <<"\n";
	// Applies transformation of the current node to the global
	// transformation matrices.
	_modelToWorld = _modelToWorld*node->trans;
	_worldToModel = node->invtrans*_worldToModel; 
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {
			//std::cout << "ray intersected with node  " << node << "\n";
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices.
	_worldToModel = node->trans*_worldToModel;
	_modelToWorld = _modelToWorld*node->invtrans;
}

void Raytracer::computeShading( Ray3D& ray ) {
	LightListNode* curLight = _lightSource;
	for (;;) {
			if (curLight == NULL) break;
			// Each lightSource provides its own shading function.

			// Implement shadows here if needed.
			Ray3D shadowRay;
			Point3D lightPos= (curLight->light)->get_position();
			shadowRay.origin = lightPos;
			shadowRay.dir = ray.intersection.point - lightPos;
			traverseScene(_root, shadowRay);
			if (shadowRay.intersection.t_value > 0.000001 && shadowRay.intersection.t_value < 0.999)
			{
				ray.inShadow = true;
			}
			switch (_mode){
				case(SIGNATURE):
					curLight->light->shadeSig(ray);
					break;
				case(PHONG):
				case(FULL):
					curLight->light->shade(ray);
					break;
				case(DIFFUSE):
					curLight->light->shadeDiffuse(ray);
					break;
			}

			curLight = curLight->next;
		}
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray , int depth) {
	Colour col(0.0, 0.0, 0.0); 
	traverseScene(_root, ray); 
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	Intersection intersection = ray.intersection;
	//std::cout << "Intersection " << intersection.point <<"\n";
	//std::cout << "Intersection reflection coef " << intersection.mat->reflection_coef <<"\n";
	if (!intersection.none) {
		//std::cout << "Shading Ray " << ray.origin << ", " << ray.dir << "\n";
		computeShading(ray);
		if (_mode == FULL && intersection.mat->reflection_coef > 0.0  && depth < _maxDepth){
			Vector3D dir = ray.dir;
			Vector3D normal = intersection.normal;

			Vector3D reflDir = -(2*(dir.dot(normal)*normal)) + dir;
			Ray3D reflection(intersection.point, reflDir);
			Colour reflCol = shadeRay(reflection, depth+1);
			//std::cout << "reflected color is " << reflCol << "\n";
			ray.col = ray.col +  (intersection.mat->reflection_coef) * reflCol;
		}
		if (_mode == FULL && intersection.mat->opacity > 0 && depth < _maxDepth){
			Vector3D inDir = ray.dir;
			Vector3D normal = intersection.normal;

			double cosIn = inDir.dot(normal);
			double sinIn = sqrt(1 - cosIn*cosIn);
			double n1, n2;
			double ratio, sinOut, cosOut;
			if (cosIn < 0){// Ray pointing into the surface
				// We assume here that the rays will only go between
				// air and objects
				n2 = intersection.mat->lightSpeed;
				n1 = 1;

			}else{
				// Ray pointing out
				n1 = intersection.mat->lightSpeed;
				n2 = 1;
			}
			ratio = n1 / n2;
			double sinT2 = ratio * ratio * (1.0 - cosIn * cosIn);
			if (sinT2 <= 1.0){
				Vector3D refrDir = ratio * inDir - (ratio + sqrt(1.0 - sinT2)) * normal;
				Ray3D refraction(intersection.point, refrDir);
				Colour refrCol = shadeRay(refraction, depth+1);
				ray.col = ray.col + (intersection.mat->opacity) * refrCol;
			}//else we have total internal reflection

		}

		col = ray.col;
		col.clamp();
	}

	return col; 
}	



void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	_maxDepth = 100;
	bool _doAntiAlias = true;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	// Construct a ray for each pixel.
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {

			// Sets up ray origin and direction in view space, 
			// image plane is at z = -1.
			Point3D origin(0, 0, 0);
			Point3D imagePlane;
			if (_mode == FULL && _doAntiAlias){
				for (float fragi = i;fragi < i + 1.0f; fragi += 0.5f){
					for (float fragj = j;fragj < j + 1.0f; fragj += 0.5f){
						imagePlane[0] = (-double(width)/2 + 0.5 + fragj)/factor;
						imagePlane[1] = (-double(height)/2 + 0.5 + fragi)/factor;
						imagePlane[2] = -1;

						Point3D rayOriginWorld = viewToWorld * imagePlane;
						Vector3D rayDirWorld = rayOriginWorld - eye;
						rayDirWorld.normalize();

						//Ray3D ray;
						Ray3D ray(rayOriginWorld, rayDirWorld);


						Colour col = shadeRay(ray, 0);

						_rbuffer[i*width+j] += int(col[0]*255*0.25f);
						_gbuffer[i*width+j] += int(col[1]*255*0.25f);
						_bbuffer[i*width+j] += int(col[2]*255*0.25f);
					}
				}
			}else{
				imagePlane[0] = (-double(width)/2 + 0.5+j)/factor;
				imagePlane[1] = (-double(height)/2 + 0.5+i)/factor;
				imagePlane[2] = -1;

				Point3D rayOriginWorld = viewToWorld * imagePlane;
				Vector3D rayDirWorld = rayOriginWorld - eye;
				rayDirWorld.normalize();

				//Ray3D ray;
				Ray3D ray(rayOriginWorld, rayDirWorld);


				Colour col = shadeRay(ray, 0);

				_rbuffer[i*width+j] = int(col[0]*255);
				_gbuffer[i*width+j] = int(col[1]*255);
				_bbuffer[i*width+j] = int(col[2]*255);
			}
		}
		if (i%20 == 0){
			std::cout << "Row "<< i << " shaded.\n";
		}
	}

	flushPixelBuffer(fileName);
}

void Raytracer::setMode(int mode){
	_mode = mode;
}

int Raytracer::getMode(){
	return _mode;
}

int main(int argc, char* argv[])
{	
	
	Raytracer raytracer;
	int width = 800;
	int height = 600;

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	Point3D eye(0, 0, 1);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	Point3D eye2(4, 2, 1);
	Vector3D view2(-4, -2, -6);
	double fov = 100;

	// Defines a material for shading.
	Material orange( Colour(0.6, 0.3, 0.1), Colour(0.75164, 0.60648, 0.22648), 
			Colour(0.628281, 0.555802, 0.366065), 
			51.2, 0.5, 0.0, 0.5);

	Material red( Colour(0, 0, 0), Colour(1.0, 0.2, 0.2),
			Colour(0.4, 0.4, 0.4), 
			12.8, 0.5, 0.0, 0.5);
			
	Material pearl(Colour(0.1, 0.1, 0.1), Colour(1.0, 1.0, 1.0), Colour(0.6, 0.6, 0.6),
			0.2, 0.7, 0.05, 0.95);
			
	Material glass(Colour(0.1, 0.1, 0.1), Colour(1.0, 1.0, 1.0), Colour(0.6, 0.6, 0.6),
			0.2, 0.96, 0.05, 0.95);
			
	Material mirror( Colour(0.05, 0.05, 0.05), Colour(0.05, 0.05, 0.05),
			Colour(0, 0, 0), 90, 1.0, 1.0, 0.75);
			
	/* Material( Colour ambient, Colour diffuse, Colour specular, double exp,
			double refl , double opac, double ls) */

	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(0, 0, 5), Colour(0.9, 0.9, 0.9) ) );
	raytracer.addLightSource( new PointLight(Point3D(4, 5, 5),
				Colour(0.2, 0.2, 0.2) ) );

	// Add a unit square into the scene with material mat.
	SceneDagNode* sphere1 = raytracer.addObject( new UnitSphere(), &pearl );
	SceneDagNode* sphere2 = raytracer.addObject( new UnitSphere(), &orange );
	SceneDagNode* sphere3 = raytracer.addObject( new UnitSphere(), &red );
	SceneDagNode* sphere4 = raytracer.addObject( new UnitSphere(), &pearl );	
	
	SceneDagNode* plane1 = raytracer.addObject( new UnitSquare(), &red );
	SceneDagNode* plane2 = raytracer.addObject( new UnitSquare(), &pearl );
	
	SceneDagNode* cylinder1 = raytracer.addObject(new UnitCylinder(), &pearl);
	
	// Apply some transformations to the unit square.
	double sphere_size[3] = { 1.0, 1.0, 1.0 };
	double cylinder_size[3] = { 2.0, 2.0, 2.0 };
	double plane_size[3] = { 8.0, 8.0, 8.0 };

	raytracer.translate(sphere1, Vector3D(0, 0.8, -4));
	raytracer.scale(sphere1, Point3D(0, 0, 0), sphere_size);
	
	raytracer.translate(sphere2, Vector3D(3, 0.8, -4));
	raytracer.scale(sphere2, Point3D(0, 0, 0), sphere_size);
	
	raytracer.translate(sphere3, Vector3D(-3, 0.8, -4));
	raytracer.scale(sphere3, Point3D(0, 0, 0), sphere_size);
	
	raytracer.translate(sphere4, Vector3D(-6, 0.8, -4));
	raytracer.scale(sphere4, Point3D(0, 0, 0), sphere_size);
	
	raytracer.translate(cylinder1, Vector3D(8,0.8,-7));
	raytracer.scale(cylinder1, Point3D(0,0,0), cylinder_size);
	
	raytracer.translate(plane1, Vector3D(0, 0, -7));
	raytracer.rotate(plane1, 'x', -80);
	raytracer.scale(plane1, Point3D(0, 0, 0), plane_size);
	
	raytracer.translate(plane2, Vector3D(0, 0, -14));
	raytracer.rotate(plane2, 'x', -45);
	raytracer.scale(plane2, Point3D(0, 0, 0), plane_size);

	raytracer.setMode(Raytracer::FULL);

	raytracer.render(width, height, eye, view, up, fov, "full1.bmp");
	std::cout << "---------------------------------full1.bmp rendered\n";
	// Render it from a different point of view.


	raytracer.render(width, height, eye2, view2, up, fov, "full2.bmp");
	std::cout << "---------------------------------full2.bmp rendered\n";



	return 0;
}

