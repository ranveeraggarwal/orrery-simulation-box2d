/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */
#include "cs296_base.hpp"
#include "render.hpp"
#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif
#include <cstring>
using namespace std;
#include "dominos.hpp"
namespace cs296
{
  /**  The is the constructor.
   * It constructs the world by setting up various objects and adding them to the world.
   */ 
  
  
  dominos_t::dominos_t()
  {


    //!Ground
    /** b2Body* ground \n
     * \brief pointer to the body. \n
     * VAR sim_t* sim \n
     * \brief pointer to the simulation */
    /** \var */
    
    b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(50.0f, 0.0f), b2Vec2(-50.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}
    
<<<<<<< HEAD

		b2RevoluteJoint* m_joint1;
		b2RevoluteJoint* m_joint2;
		b2Joint* m_joint3;
		b2GearJoint* m_joint4;
		b2GearJoint* m_joint5;
		b2WeldJoint* m_joint6;
		b2Joint* x;

		{
			b2CircleShape circle1;//!Gear 1 shape left
			circle1.m_radius = 2.0f; //!Gear1 

			b2CircleShape circle2; //!gear2 shape right
			circle2.m_radius = 2.0f;//!gear2 radius
			
			b2PolygonShape box;//rod shape right
			box.SetAsBox(0.5f, 5.0f);

			b2PolygonShape rod;//rod shape left
			rod.SetAsBox(0.5f, 5.0f);

			b2BodyDef bd1; //! left  gear
=======
		//Joints
		b2RevoluteJoint* rjoint1a;
		b2RevoluteJoint* rjoint2a;
		b2PrismaticJoint* pjoint2a;
		b2GearJoint* gjoint12;
		b2GearJoint* gjoint23;
		b2WeldJoint* wjoint1b;
		b2WeldJoint* wjoint1a;

		//Shape initializations
		{
			//The circle on the left
			b2CircleShape circle1;
			circle1.m_radius = 2.0f;

			//The circle on the right
			b2CircleShape circle2;
			circle2.m_radius = 2.0f;

			//The small circle attached to circle 1
			b2CircleShape circle1a;
			circle1a.m_radius = 1.0f;
			
			//The rod that initiates the whole thing
			b2PolygonShape box;
			box.SetAsBox(0.5f, 5.0f);

			//The rod connected to circle 1
			b2PolygonShape rod1a;
			rod1a.SetAsBox(0.5f, 5.0f);
		
			//Circle 1 definition -- body 1 (circle1)
			b2BodyDef bd1;
>>>>>>> 5ca7519e5911f53a03dd3b29254c51718b186add
			bd1.type = b2_dynamicBody;
			bd1.position.Set(-10.0f, 11.0f);
			b2Body* body1 = m_world->CreateBody(&bd1);
			body1->CreateFixture(&circle1, 5.0f);

<<<<<<< HEAD
			

			b2BodyDef bd2; //! right gear
=======
			//The smaller circle attached to circle 1 -- body1a (circle1a)
			b2BodyDef bd1a;
			bd1a.type = b2_dynamicBody;
			bd1a.position.Set(-10.0f, 12.0f);
			b2Body* body1a = m_world->CreateBody(&bd1a);
			body1a->CreateFixture(&circle1a, 5.0f);

			//Weld circle1 and circle1a together -- wjoint1a
			b2WeldJointDef jdw1a;
			jdw1a.bodyA = body1a;
			jdw1a.bodyB = body1;
			wjoint1a = (b2WeldJoint*)m_world->CreateJoint(&jdw1a);

			//Revolute joint between circle1 and ground -- rjoint1a
			b2RevoluteJointDef jd1;
			jd1.bodyA = ground;
			jd1.bodyB = body1;
			jd1.localAnchorA = ground->GetLocalPoint(bd1.position);
			jd1.localAnchorB = body1->GetLocalPoint(bd1.position);
			jd1.referenceAngle = body1->GetAngle() - ground->GetAngle();
			rjoint1a = (b2RevoluteJoint*)m_world->CreateJoint(&jd1);

			//Circle 2 definition -- body 2 (circle2)
			b2BodyDef bd2;
>>>>>>> 5ca7519e5911f53a03dd3b29254c51718b186add
			bd2.type = b2_dynamicBody;
			bd2.position.Set(0.0f, 12.0f);
			b2Body* body2 = m_world->CreateBody(&bd2);
			body2->CreateFixture(&circle2, 5.0f);

<<<<<<< HEAD
			
			//right box
=======
			//Revolute joint between circle2 and ground -- rjoint2a
			b2RevoluteJointDef jd2;
			jd2.Initialize(ground, body2, bd2.position);
			rjoint2a = (b2RevoluteJoint*)m_world->CreateJoint(&jd2);

			//The box intitializer defined -- body 3 (box)
>>>>>>> 5ca7519e5911f53a03dd3b29254c51718b186add
			b2BodyDef bd3;
			bd3.type = b2_dynamicBody;
			bd3.position.Set(2.5f, 12.0f);
			b2Body* body3 = m_world->CreateBody(&bd3);
			body3->CreateFixture(&box, 5.0f);

<<<<<<< HEAD

			//left box
			b2BodyDef bdx;
			bdx.type = b2_dynamicBody;
			bdx.position.Set(-10.0f, 13.0f);
			b2Body* bodyx = m_world->CreateBody(&bdx);
			bodyx->CreateFixture(&rod, 5.0f);
			//Joint between ground and left box
			
			b2RevoluteJointDef jd2;
			jd2.Initialize(ground, bodyx, bdx.position);
			m_world->CreateJoint(&jd2);
			//Joint between left box and left gear
			b2RevoluteJointDef jd3;
			jd3.Initialize(bodyx, body1, bd1.position);
			m_joint3 = m_world->CreateJoint(&jd3);
			
			//Joint bw right box and ground
			b2PrismaticJointDef jdp;
			jdp.Initialize(ground, body3, bd3.position, b2Vec2(0.0f, 1.0f));
			jdp.lowerTranslation = -5.0f;
			jdp.upperTranslation = 5.0f;
			jdp.enableLimit = true;
			x = m_world->CreateJoint(&jdp);
			
			//Right gear and ground joint
			b2RevoluteJointDef jdrg;
			jdrg.Initialize(ground, body2, bd2.position);
			m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jdrg);
			
			b2GearJointDef jd4;
			jd4.bodyA = body2;
			jd4.bodyB = body3;
			jd4.joint1 =x;
			jd4.joint2 = m_joint2;
			jd4.ratio = 1;
			(b2GearJoint*)m_world->CreateJoint(&jd4);
			
			b2GearJointDef jd5;
			jd5.bodyA = body1;
			jd5.bodyB = body2;
			jd5.joint1 = m_joint2;
			jd5.joint2 = m_joint3;
			jd5.ratio = 1;
			m_joint5 = (b2GearJoint*)m_world->CreateJoint(&jd5);

			/*b2RevoluteJointDef jd1;
			jd1.bodyA = ground;
			jd1.bodyB = bodyx;
			jd1.localAnchorA = ground->GetLocalPoint(bdx.position);
			jd1.localAnchorB = bodyx->GetLocalPoint(bdx.position);
			jd1.referenceAngle = bodyx->GetAngle() - ground->GetAngle();
			m_joint1 = (b2RevoluteJoint*)m_world->CreateJoint(&jd1);
 
=======
			//The rod defined -- body 1b (rod1a)
			b2BodyDef bd1b;
			bd1b.type = b2_dynamicBody;
			bd1b.position.Set(-10.0f, 13.0f);
			b2Body* body1b = m_world->CreateBody(&bd1b);
			body1b->CreateFixture(&rod1a, 5.0f);

			//The weld joint between circle 1 and the rod1a
			b2WeldJointDef jdx;
>>>>>>> 5ca7519e5911f53a03dd3b29254c51718b186add
			jdx.bodyA = body1;
			jdx.bodyB = body1b;
			wjoint1b = (b2WeldJoint*)m_world->CreateJoint(&jdx);

<<<<<<< HEAD
			
=======
			//Prismatic joint between ground and intitial box
			b2PrismaticJointDef jd3;
			jd3.Initialize(ground, body3, bd3.position, b2Vec2(0.0f, 1.0f));
			jd3.lowerTranslation = -5.0f;
			jd3.upperTranslation = 5.0f;
			jd3.enableLimit = true;
>>>>>>> 5ca7519e5911f53a03dd3b29254c51718b186add

			pjoint2a = (b2PrismaticJoint*)m_world->CreateJoint(&jd3);

			//The gear joint between circle 1 and circle 2
			b2GearJointDef jd4;
			jd4.bodyA = body1;
			jd4.bodyB = body2;
			jd4.joint1 = rjoint1a;
			jd4.joint2 = rjoint2a;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			gjoint12 = (b2GearJoint*)m_world->CreateJoint(&jd4);
			
			//The gear joint between circle 2 and box
			b2GearJointDef jd5;
			jd5.bodyA = body2;
			jd5.bodyB = body3;
			jd5.joint1 = rjoint2a;
			jd5.joint2 = pjoint2a;
			jd5.ratio = -1.0f / circle2.m_radius;
<<<<<<< HEAD
			m_joint5 = (b2GearJoint*)m_world->CreateJoint(&jd5);*/
=======
			gjoint23 = (b2GearJoint*)m_world->CreateJoint(&jd5);
>>>>>>> 5ca7519e5911f53a03dd3b29254c51718b186add
		}
    
    	
    
  }

	
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
