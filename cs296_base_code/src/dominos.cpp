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





	b2Body* dominos_t::createGear(float r, uint16 layer, float x, float y)
	{
		r = r;
		int n = int(5 * r);
		r = n/5.0 ;
		r = r - 0.3;
		b2CircleShape gear_shape;
		gear_shape.m_radius = r;
		b2BodyDef gear_bd;
		gear_bd.type = b2_dynamicBody;
		gear_bd.position.Set(x, y);
		b2Body *gear_b = m_world->CreateBody(&gear_bd);
		b2FixtureDef gear_body_fd;
		gear_body_fd.filter.categoryBits = 0x0010;
		gear_body_fd.filter.maskBits = 0x0000;
		gear_body_fd.shape = &gear_shape;
		gear_body_fd.density = 1.0f;
		gear_b->CreateFixture(&gear_body_fd);

		for(int i=0; i<2*n; i++)
		{

			float pi = 3.151592654;
			b2PolygonShape gear_tooth;

			float xc = (r+0.2);// * cos((pi*i)/n);
			float yc = 0;//* sin((pi*i)/n);
			b2Vec2 vertices[5];
			float xt, yt ;
			float ct=cos((pi*i)/n), st =  sin((pi*i)/n);

			xt = xc-0.2; yt = yc +0.1;
			vertices[0].Set(xt*ct + yt*st, yt*ct - xt*st);
			xt = xc + 0.1; yt = yc +0.1;

			vertices[1].Set(xt*ct + yt*st, yt*ct - xt*st);
			xt = xc + 0.2; yt = yc;
			vertices[2].Set(xt*ct + yt*st, yt*ct - xt*st);
			xt = xc + 0.1; yt = yc -0.1;
			vertices[3].Set(xt*ct + yt*st, yt*ct - xt*st);
			xt = xc - 0.2; yt = yc -0.1;
			vertices[4].Set(xt*ct + yt*st, yt*ct - xt*st);
			gear_tooth.Set(vertices, 5);

			//gear_tooth.SetAsBox(0.2, 0.1, b2Vec2((r+0.2) * cos((pi*i)/n), (r+0.2)* sin((pi*i)/n)) , (pi*i) / n);
			b2FixtureDef tooth_fd;
			tooth_fd.filter.categoryBits = layer;
			tooth_fd.filter.maskBits =  layer;
			tooth_fd.shape = &gear_tooth;
			tooth_fd.density = 5.0f;
			gear_b->CreateFixture(&tooth_fd);
		}
		return gear_b;

	}

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
		b2Body* dgs_b4;
		b2RevoluteJoint * dgs_joint, *dgs_joint2, *dgs_joint3;
		
		uint16 GROUND = 0x0001, GEAR_LAYER1 = 0x0002, GEAR_LAYER2 = 0x0004, 
				GEAR_LAYER3 = 0x0008, GEAR_LAYER4 = 0x0010 , GEAR_LAYER5 = 0x0020, GEAR_LAYER6 = 0x0040, NC_LAYER = 0x0010;

		//! Main driver gear
		{
			dgs_b1 = createGear(5, GEAR_LAYER3, -15, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = dgs_b1;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = dgs_b1->GetLocalPoint(dgs_b1->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(dgs_b1->GetPosition());
			g1_jd.enableMotor = true;
			g1_jd.motorSpeed = 0.5;
			g1_jd.maxMotorTorque = 1000000;

			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
		}
		//!Driver gear 2
		{
			dgs_b2 = createGear(4, GEAR_LAYER2, -15, 20);
			b2WeldJointDef wdj;
			wdj.Initialize(dgs_b1, dgs_b2, dgs_b2->GetPosition());
			m_world->CreateJoint(&wdj);
		}
		//! Driver gear 3
		{
			dgs_b3 = createGear(3, GEAR_LAYER1, -15, 20);
			b2WeldJointDef wdj;
			wdj.Initialize(dgs_b2, dgs_b3, dgs_b3->GetPosition());
			m_world->CreateJoint(&wdj);
		}
		//! Driver gear 4
		{
			dgs_b4 = createGear(6, GEAR_LAYER4, -15, 20);
			b2WeldJointDef wdj;
			wdj.Initialize(dgs_b3, dgs_b4, dgs_b4->GetPosition());
			m_world->CreateJoint(&wdj);
		}
		//! Moon magnification gear
		{
			b2Body *b = createGear(1, GEAR_LAYER4, -15, 13);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);

			b2Body* b1 = createGear(5, GEAR_LAYER5, -15, 13);
			b2WeldJointDef wd_jd;
			wd_jd.Initialize(b, b1, b1->GetPosition());
			m_world->CreateJoint(&wd_jd);
			
			
		}
		//! moon conn gear
		{
			b2Body *b = createGear(4, GEAR_LAYER5, -6.98, 16.742);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);

			
			
			
			
		}
		//!Moon gear
		{
			b2Body *b = createGear(3.9, GEAR_LAYER5, 0, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			b2Body *b1 = createGear(3, GEAR_LAYER6, 0, 20);
			b2WeldJointDef wdj ;
			wdj.Initialize(b, b1, b1->GetPosition());
			m_world->CreateJoint(&wdj);

			
			
			
		}

		//! Earth gear
		{
			b2Body *b = createGear(5, GEAR_LAYER1, 0, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			/*g1_jd.enableMotor = true;
			g1_jd.motorSpeed = 0;
			g1_jd.maxMotorTorque = 1000;*/
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			b2PolygonShape rod_s;
			rod_s.SetAsBox(15, 0.5, b2Vec2(13, 0), 0);
			b2FixtureDef rod_fd;
			rod_fd.shape = &rod_s;
			rod_fd.density = 1.0f;
			rod_fd.filter.categoryBits = NC_LAYER;
			rod_fd.filter.maskBits = GROUND;
			b->CreateFixture(&rod_fd);

			b2Body *conn_gear1 = createGear(3.2, GEAR_LAYER6, 6, 20);
			b2RevoluteJointDef cg_rjd1;
			cg_rjd1.bodyA = b;
			cg_rjd1.bodyB = conn_gear1;
			cg_rjd1.localAnchorA = b->GetLocalPoint(conn_gear1->GetPosition());
			cg_rjd1.localAnchorB = conn_gear1->GetLocalPoint(conn_gear1->GetPosition());
			m_world->CreateJoint(&cg_rjd1);

			b2Body *conn_gear2 = createGear(3.2, GEAR_LAYER6, 12.3, 20);
			b2RevoluteJointDef cg_rjd2;
			cg_rjd2.bodyA = b;
			cg_rjd2.bodyB = conn_gear2;
			cg_rjd2.localAnchorA = b->GetLocalPoint(conn_gear2->GetPosition());
			cg_rjd2.localAnchorB = conn_gear2->GetLocalPoint(conn_gear2->GetPosition());
			m_world->CreateJoint(&cg_rjd2);

			b2Body *conn_gear3 = createGear(3.2, GEAR_LAYER6, 18.5, 20);
			b2RevoluteJointDef cg_rjd3;
			cg_rjd3.bodyA = b;
			cg_rjd3.bodyB = conn_gear3;
			cg_rjd3.localAnchorA = b->GetLocalPoint(conn_gear3->GetPosition());
			cg_rjd3.localAnchorB = conn_gear3->GetLocalPoint(conn_gear3->GetPosition());
			m_world->CreateJoint(&cg_rjd3);

			
		}
		//! Earth conn gear
		{
			b2Body *b = createGear(3.6, GEAR_LAYER1, -8.5, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);

			
			
			
		}


		

		/*{
			dgs_b2 = createGear(1, GEAR_LAYER1, -9, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = dgs_b2;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = dgs_b2->GetLocalPoint(dgs_b2->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(dgs_b2->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			
			
		}

		{
			dgs_b2 = createGear(3, GEAR_LAYER1, -5, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = dgs_b2;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = dgs_b2->GetLocalPoint(dgs_b2->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(dgs_b2->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			
			
		}

		{
			dgs_b2 = createGear(5, GEAR_LAYER1, 3, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = dgs_b2;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = dgs_b2->GetLocalPoint(dgs_b2->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(dgs_b2->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			
			
		}

		{
			dgs_b2 = createGear(5, GEAR_LAYER2, -15, 12);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = dgs_b2;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = dgs_b2->GetLocalPoint(dgs_b2->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(dgs_b2->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			
			
		}
		*/


    	
    
  }

	
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
