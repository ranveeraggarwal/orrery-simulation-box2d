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
			bd1.type = b2_dynamicBody;
			bd1.position.Set(-10.0f, 11.0f);
			b2Body* body1 = m_world->CreateBody(&bd1);
			body1->CreateFixture(&circle1, 5.0f);

			

			b2BodyDef bd2; //! right gear
			bd2.type = b2_dynamicBody;
			bd2.position.Set(0.0f, 12.0f);
			b2Body* body2 = m_world->CreateBody(&bd2);
			body2->CreateFixture(&circle2, 5.0f);

			
			//right box
			b2BodyDef bd3;
			bd3.type = b2_dynamicBody;
			bd3.position.Set(2.5f, 12.0f);
			b2Body* body3 = m_world->CreateBody(&bd3);
			body3->CreateFixture(&box, 5.0f);


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
 
			jdx.bodyA = body1;
			jdx.bodyB = bodyx;
			m_joint6 = (b2WeldJoint*)m_world->CreateJoint(&jdx);

			

			m_joint3 = (b2PrismaticJoint*)m_world->CreateJoint(&jd3);

			b2GearJointDef jd4;
			jd4.bodyA = body1;
			jd4.bodyB = body2;
			jd4.joint1 = m_joint1;
			jd4.joint2 = m_joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			m_joint4 = (b2GearJoint*)m_world->CreateJoint(&jd4);

			b2GearJointDef jd5;
			jd5.bodyA = body2;
			jd5.bodyB = body3;
			jd5.joint1 = m_joint2;
			jd5.joint2 = m_joint3;
			jd5.ratio = -1.0f / circle2.m_radius;
			m_joint5 = (b2GearJoint*)m_world->CreateJoint(&jd5);*/
		}
    
    	
    
  }

	
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
