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
    
		b2Body* dgs_b1;
		b2Body* dgs_b2;
		b2Body* dgs_b3;
		b2RevoluteJoint * dgs_joint, *dgs_joint2, *dgs_joint3;
		
		uint16 GROUND = 0x0001, GEAR_LAYER1 = 0x0002, GEAR_LAYER2 = 0x0004, GEAR_LAYER3 = 0x0008, GEAR_LAYER4 = 0x0010, NC_LAYER = 0x0010;
		//! The three coupled driver gears
		{
			b2CircleShape dgs_shp1; //Body shapes : Circles
			dgs_shp1.m_radius = 5.0f;
			b2CircleShape dgs_shp2;
			dgs_shp2.m_radius = 3.0f;
			b2CircleShape dgs_shp3;
			dgs_shp3.m_radius = 1.0f; 


			b2BodyDef dgs_bd1, dgs_bd2, dgs_bd3;  //The body defs
			dgs_bd1.type = b2_dynamicBody;
			dgs_bd2.type = b2_dynamicBody;
			dgs_bd3.type = b2_dynamicBody;
			dgs_bd1.position.Set(-15.0f, 20.0f);
			dgs_bd2.position.Set(-15.0f, 20.0f);
			dgs_bd3.position.Set(-15.0f, 20.0f);
			dgs_b1 = m_world->CreateBody(&dgs_bd1); //The gear bodies
			dgs_b2 = m_world->CreateBody(&dgs_bd2);
			dgs_b3 = m_world->CreateBody(&dgs_bd3);
			b2FixtureDef dgs_fd1, dgs_fd2, dgs_fd3;
			dgs_fd1.shape = &dgs_shp1;
			dgs_fd1.density = 5.0f;
			dgs_fd1.filter.categoryBits = GEAR_LAYER1;
			dgs_fd1.filter.maskBits =  GROUND | GEAR_LAYER1;
			dgs_fd2.shape = &dgs_shp2;
			dgs_fd2.density = 5.0f;
			dgs_fd2.filter.categoryBits = GEAR_LAYER2;
			dgs_fd2.filter.maskBits =  GROUND | GEAR_LAYER2;
			dgs_fd3.shape = &dgs_shp3;
			dgs_fd3.density = 5.0f;
			dgs_fd3.filter.categoryBits = GEAR_LAYER3;
			dgs_fd3.filter.maskBits = GROUND |GEAR_LAYER3;
			dgs_b1->CreateFixture(&dgs_fd1);
			dgs_b2->CreateFixture(&dgs_fd2);
			dgs_b3->CreateFixture(&dgs_fd3);

			b2RevoluteJointDef dgs_joint_def;
			dgs_joint_def.bodyA = ground;
			dgs_joint_def.bodyB = dgs_b1;

			dgs_joint_def.enableMotor = true;
			dgs_joint_def.motorSpeed = 1;
			dgs_joint_def.maxMotorTorque = 10000;

			dgs_joint_def.localAnchorA = ground->GetLocalPoint(dgs_bd1.position);
			dgs_joint_def.localAnchorB = dgs_b1->GetLocalPoint(dgs_bd1.position);
			dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&dgs_joint_def);

			b2RevoluteJointDef dgs_joint_def2;
			dgs_joint_def2.bodyA = ground;
			dgs_joint_def2.bodyB = dgs_b2;

			dgs_joint_def2.localAnchorA = ground->GetLocalPoint(dgs_bd2.position);
			dgs_joint_def2.localAnchorB = dgs_b1->GetLocalPoint(dgs_bd2.position);
			dgs_joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&dgs_joint_def2);

			b2RevoluteJointDef dgs_joint_def3;
			dgs_joint_def3.bodyA = ground;
			dgs_joint_def3.bodyB = dgs_b3;

			dgs_joint_def3.localAnchorA = ground->GetLocalPoint(dgs_bd3.position);
			dgs_joint_def3.localAnchorB = dgs_b1->GetLocalPoint(dgs_bd3.position);
			dgs_joint3 = (b2RevoluteJoint*)m_world->CreateJoint(&dgs_joint_def3);
			

			b2GearJointDef glink_jd12;
			glink_jd12.bodyA = dgs_b1;
			glink_jd12.bodyB = dgs_b2;
			glink_jd12.joint1 = dgs_joint;
			glink_jd12.joint2 = dgs_joint2;
			glink_jd12.ratio = -1;
			glink_jd12.collideConnected = false;
			(b2GearJoint*)m_world->CreateJoint(&glink_jd12);

			b2GearJointDef glink_jd23;
			glink_jd12.bodyA = dgs_b2;
			glink_jd12.bodyB = dgs_b3;
			glink_jd12.joint1 = dgs_joint2;
			glink_jd12.joint2 = dgs_joint3;
			glink_jd12.ratio = -1;
			glink_jd12.collideConnected = false;
			(b2GearJoint*)m_world->CreateJoint(&glink_jd12);
		}

		//!Closest to sun planet
		{
			b2CircleShape circ;
			circ.m_radius = 1.0f;
			b2PolygonShape rod;
			rod.SetAsBox(5, 0.1);
			b2BodyDef gear_bdef;
			gear_bdef.type = b2_dynamicBody;
			gear_bdef.position.Set(-9.0, 20);
			b2Body* gear_b = m_world->CreateBody(&gear_bdef);
			b2FixtureDef circ_fd;
			circ_fd.shape = &circ;
			circ_fd.density = 5.0f;
			circ_fd.filter.categoryBits =GEAR_LAYER1;
			circ_fd.filter.maskBits = GROUND | GEAR_LAYER1;
			gear_b->CreateFixture(&circ_fd);
			b2BodyDef rod_bdef;
			rod_bdef.type = b2_dynamicBody;
			rod_bdef.position.Set(-4.0, 20);
			b2Body* rod_b = m_world->CreateBody(&rod_bdef);
			b2FixtureDef rod_fd;
			rod_fd.shape = &rod;
			rod_fd.density = 5.0f;
			rod_fd.filter.categoryBits = NC_LAYER;
			rod_fd.filter.maskBits = GROUND ;
			rod_b->CreateFixture(&rod_fd);
			b2RevoluteJointDef gr_jdef;
			gr_jdef.bodyA = gear_b;
			gr_jdef.bodyB = ground;
			gr_jdef.localAnchorA = gear_b->GetLocalPoint(gear_bdef.position);
			gr_jdef.localAnchorB = ground->GetLocalPoint(gear_bdef.position);
			b2RevoluteJoint *gr_j = (b2RevoluteJoint*) m_world->CreateJoint(&gr_jdef);
			b2WeldJointDef gr_weld_def;
			gr_weld_def.Initialize(gear_b, rod_b, gear_b->GetWorldCenter());
			gr_weld_def.collideConnected = false;
			m_world->CreateJoint(&gr_weld_def);
			b2GearJointDef glink_jd1;
			glink_jd1.bodyA = dgs_b1;
			glink_jd1.bodyB = gear_b;
			glink_jd1.joint1 = dgs_joint;
			glink_jd1.joint2 = gr_j;
			glink_jd1.ratio = -0.2;
			glink_jd1.collideConnected = false;
			(b2GearJoint*)m_world->CreateJoint(&glink_jd1);

		}

		{
			b2CircleShape circ;
			circ.m_radius = 3.0f;
			b2PolygonShape rod;
			rod.SetAsBox(10, 0.1);
			b2BodyDef gear_bdef;
			gear_bdef.type = b2_dynamicBody;
			gear_bdef.position.Set(-9.0, 20);
			b2Body* gear_b = m_world->CreateBody(&gear_bdef);
			b2FixtureDef circ_fd;
			circ_fd.shape = &circ;
			circ_fd.density = 5.0f;
			circ_fd.filter.categoryBits =GEAR_LAYER2;
			circ_fd.filter.maskBits = GROUND | GEAR_LAYER2;
			gear_b->CreateFixture(&circ_fd);
			b2BodyDef rod_bdef;
			rod_bdef.type = b2_dynamicBody;
			rod_bdef.position.Set(1.0, 20);
			b2Body* rod_b = m_world->CreateBody(&rod_bdef);
			b2FixtureDef rod_fd;
			rod_fd.shape = &rod;
			rod_fd.density = 5.0f;
			rod_fd.filter.categoryBits = NC_LAYER;
			rod_fd.filter.maskBits = GROUND;
			rod_b->CreateFixture(&rod_fd);
			b2RevoluteJointDef gr_jdef;
			gr_jdef.bodyA = gear_b;
			gr_jdef.bodyB = ground;
			gr_jdef.localAnchorA = gear_b->GetLocalPoint(gear_bdef.position);
			gr_jdef.localAnchorB = ground->GetLocalPoint(gear_bdef.position);
			b2RevoluteJoint *gr_j = (b2RevoluteJoint*) m_world->CreateJoint(&gr_jdef);
			b2WeldJointDef gr_weld_def;
			gr_weld_def.Initialize(gear_b, rod_b, gear_b->GetWorldCenter());
			gr_weld_def.collideConnected = false;
			m_world->CreateJoint(&gr_weld_def);
			b2GearJointDef glink_jd1;
			glink_jd1.bodyA = dgs_b2;
			glink_jd1.bodyB = gear_b;
			glink_jd1.joint1 = dgs_joint2;
			glink_jd1.joint2 = gr_j;
			glink_jd1.ratio = -1;
			glink_jd1.collideConnected = false;
			(b2GearJoint*)m_world->CreateJoint(&glink_jd1);

		}





			/*
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

	
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
