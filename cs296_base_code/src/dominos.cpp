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




	//!Function to create gear of radius ~r. 
	b2Body* dominos_t::createGear(float r, uint16 layer, float x, float y, float density = 1.0f)
	{
		r = r;
		int n = int(5 * r);
		r = n/5.0 ;
		r = r - 0.3;
		//!Gear body
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
		gear_body_fd.density = density;
		gear_b->CreateFixture(&gear_body_fd);

		for(int i=0; i<2*n; i++)
		{

			float pi = 3.151592654;
			
			//! Shape of the gear teeth
			b2PolygonShape gear_tooth;

			////////////////////////////////////////////////////////////
			//Uncomment to use square gear teeth
			/*gear_tooth.SetAsBox(0.2, 0.1, 
					b2Vec2((r+0.2) * cos((pi*i)/n), 
					(r+0.2)* sin((pi*i)/n)) , (pi*i) / n);*/
			////////////////////////////////////////////////////////////		
					
			
			////////////////////////////////////////////////////////////
			//Uncomment to use pentagon teeth.
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
			////////////////////////////////////////////////////////////
			
			//Fixture for the gear teeth.
			b2FixtureDef tooth_fd;
			tooth_fd.filter.categoryBits = layer;
			tooth_fd.filter.maskBits =  layer;
			tooth_fd.shape = &gear_tooth;
			tooth_fd.density = density;
			gear_b->CreateFixture(&tooth_fd);
		}
		return gear_b;

	}
	/**
	 * Fix b1 and b2 with a revolute joint at b1's center.
	 */
	b2RevoluteJoint* dominos_t::fixCenterRevolute(b2Body *b1, b2Body *b2, float motor_speed = 0, float motor_torque = 10)
	{
		b2RevoluteJointDef rjd;
		rjd.bodyA = b1;
		rjd.bodyB = b2;
		rjd.localAnchorA = b1->GetLocalPoint(b1->GetPosition());
		rjd.localAnchorB = b2->GetLocalPoint(b1->GetPosition());
		rjd.enableMotor = true;
		rjd.motorSpeed = motor_speed;
		rjd.maxMotorTorque = motor_torque;
		return (b2RevoluteJoint*)m_world->CreateJoint(&rjd);
	}
	b2WeldJoint* dominos_t::weld(b2Body *b1, b2Body *b2)
	{
		b2WeldJointDef wjd;
		wjd.Initialize(b1, b2, b1->GetPosition());
		return (b2WeldJoint*)m_world->CreateJoint(&wjd);
	}
	
	b2Body* dominos_t::createPlanet(float length, float radius)
	{
		
		b2PolygonShape rod;
		rod.SetAsBox(length, 0.1);
		b2FixtureDef rodfixture;
		rodfixture.shape = &rod;
		rodfixture.density = 0.1;
		rodfixture.filter.categoryBits = 0x0800;
		rodfixture.filter.maskBits = 0x0000;
		b2BodyDef rodbd;
		rodbd.position.Set(length,0);
		rodbd.type = b2_dynamicBody;
		b2Body* rodb = m_world->CreateBody(&rodbd);
		rodb->CreateFixture(&rodfixture);
		
		b2CircleShape planet;
		planet.m_radius = radius;
		b2FixtureDef planetfixture;
		planetfixture.shape = &planet;
		planetfixture.density = 0.1;
		planetfixture.filter.categoryBits = 0x0800;
		planetfixture.filter.maskBits = 0x0000;
		b2BodyDef planetbd;
		planetbd.type = b2_dynamicBody;
		planetbd.position.Set(2*length,0);
		b2Body* plb = m_world->CreateBody(&planetbd);
		plb->CreateFixture(&planetfixture);
		
		weld(rodb, plb);
		return rodb;
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
    
		
		uint16 DRIVER_GEAR_LAYER = 0x0001, MARS_GEAR_LAYER = 0x0002, EARTH_GEAR_LAYER = 0x0004, MOON1_GEAR_LAYER = 0x0008,
				MOON2_GEAR_LAYER = 0x0010, VENUS_GEAR_LAYER = 0x0020, MERCURY_GEAR_LAYER = 0x0040;
		
		
		
		////////Driver->Mars connection/////////////////////////////////
		b2Body* mars_driver_gear2 = createGear(5, MARS_GEAR_LAYER,10.85,0);
		b2RevoluteJoint* mdg2_rev_joint = fixCenterRevolute(mars_driver_gear2, ground, 0,0);
		b2Body* mars_driver_gear1 = createGear(4, DRIVER_GEAR_LAYER,10.85,0);
		b2RevoluteJoint* mdg1_rev_joint = fixCenterRevolute(mars_driver_gear1, ground, 0,0);
		b2WeldJoint* mdg_12_weld_joint = weld(mars_driver_gear1, mars_driver_gear2);
		
		
		////////Mars->Earth connection gear/////////////////////////////
		b2Body* earth_mars_gear2 = createGear(4, EARTH_GEAR_LAYER,3.74,8.02);
		b2RevoluteJoint* emg2_rev_joint = fixCenterRevolute(earth_mars_gear2, ground, 0,0);
		b2Body* earth_mars_gear1 = createGear(3, MARS_GEAR_LAYER,3.74,8.02);
		b2RevoluteJoint* emg1_rev_joint = fixCenterRevolute(earth_mars_gear1, ground, 0,0);
		b2WeldJoint* emg_12_weld_joint = weld(earth_mars_gear1, earth_mars_gear2);
		
		
		////////Earth->Venus connection gear////////////////////////////
		b2Body* venus_earth_gear2 = createGear(4.5, VENUS_GEAR_LAYER,-7.39,-4.5);
		b2RevoluteJoint* veg2_rev_joint = fixCenterRevolute(venus_earth_gear2, ground, 0,0);
		b2Body* venus_earth_gear1 = createGear(3.9, EARTH_GEAR_LAYER,-7.39,-4.5);
		b2RevoluteJoint* veg1_rev_joint = fixCenterRevolute(venus_earth_gear1, ground, 0,0);
		b2WeldJoint* veg_12_weld_joint = weld(venus_earth_gear1, venus_earth_gear2);
		////////Venus->Mercury connection gear////////////////////////////
		b2Body* merc_venus_gear2 = createGear(4, MERCURY_GEAR_LAYER,-5.13,5.13);
		b2RevoluteJoint* mvg2_rev_joint = fixCenterRevolute(merc_venus_gear2, ground, 0,0);
		b2Body* merc_venus_gear1 = createGear(3, VENUS_GEAR_LAYER,-5.13,5.13);
		b2RevoluteJoint* mvg1_rev_joint = fixCenterRevolute(merc_venus_gear1, ground, 0,0);
		b2WeldJoint* mvg_12_weld_joint = weld(merc_venus_gear1, merc_venus_gear2);
		
		
		////////Mercury gear////////////////////////////////////////////
		b2Body* mercury_gear_body = createGear(3.5, MERCURY_GEAR_LAYER, 0, 0);
		b2RevoluteJoint* mercury_gear_rev_joint = fixCenterRevolute(mercury_gear_body, ground, 0, 0);
		b2Body* mercury_body = createPlanet(9, 1);
		weld(mercury_gear_body,mercury_body);
		
		////////Venus gear///////////////////////////////////////////////
		b2Body* venus_gear_body = createGear(4.5, VENUS_GEAR_LAYER, 0, 0);
		b2RevoluteJoint* venus_gear_rev_joint = fixCenterRevolute(venus_gear_body, ground, 0, 0);
		b2Body* venus_body = createPlanet(12, 1.5);
		weld(venus_gear_body,venus_body);
		
		////////Moon gear driver////////////////////////////////////////
		b2Body* moon2_gear_body = createGear(4,MOON2_GEAR_LAYER, 0, 0);
		b2RevoluteJoint* moon2_gear_rev_joint = fixCenterRevolute(moon2_gear_body, ground, 0, 0);
		
		
		////////Moon gear driven////////////////////////////////////////
		b2Body* moon1_gear_body = createGear(3,MOON1_GEAR_LAYER, 0, 0);
		b2RevoluteJoint* moon1_gear_rev_joint = fixCenterRevolute(moon1_gear_body, ground, 0, 0);
		
		////////Moon driven driver weld/////////////////////////////////
		b2WeldJoint* moon_dri_weld = weld(moon1_gear_body, moon2_gear_body);
		
		
		////////Earth gear//////////////////////////////////////////////
		b2Body* earth_gear_body = createGear(5, EARTH_GEAR_LAYER, 0, 0);
		b2RevoluteJoint* earth_gear_rev_joint = fixCenterRevolute(earth_gear_body, ground, 0, 0);
		b2Body* earth_body = createPlanet(17, 3);
		weld(earth_gear_body,earth_body);
		
		////////Mars gear///////////////////////////////////////////////
		b2Body* mars_gear_body = createGear(6, MARS_GEAR_LAYER, 0, 0);
		b2RevoluteJoint* mars_gear_rev_joint = fixCenterRevolute(mars_gear_body, ground, 0, 0);
		b2Body* mars_body = createPlanet(23, 2.5);
		weld(mars_gear_body,mars_body);
		
		
		////////Driver gear/////////////////////////////////////////////
		b2Body* dri_gear_body = createGear(7, DRIVER_GEAR_LAYER, 0, 0);
		b2RevoluteJoint* dri_gear_rev_joint = fixCenterRevolute(dri_gear_body, ground, 1, 100000);
		
		
		////////Earth->moon acceleration gear///////////////////////////
		b2Body* earth_moon_gear2 = createGear(5.1 , MOON1_GEAR_LAYER,2.702,-7.423);
		b2RevoluteJoint* emoong2_rev_joint = fixCenterRevolute(earth_moon_gear2, ground, 0,0);
		b2Body* earth_moon_gear1 = createGear(3, EARTH_GEAR_LAYER,2.702,-7.423);
		b2RevoluteJoint* emoong1_rev_joint = fixCenterRevolute(earth_moon_gear1, ground, 0,0);
		b2WeldJoint* emoong_12_weld_joint = weld(earth_moon_gear1, earth_moon_gear2);
		
		
		
		
		
		
		
		
		
		
		/*uint16  DRIVER_GEAR_LAYER = 0x0001,  GEAR_LAYER2 = 0x0004, 
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
			b2Body *b = createGear(1.2, GEAR_LAYER4, -15, 13);
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
			g1_jd.enableMotor = true;
			g1_jd.motorSpeed = 0;
			g1_jd.maxMotorTorque = 1000;
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			b2PolygonShape rod_s;
			rod_s.SetAsBox(12.7, 0.2, b2Vec2(12.3, 0), 0);
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

			b2Body *conn_gear2 = createGear(3.2, GEAR_LAYER6, 12.2, 20);
			b2RevoluteJointDef cg_rjd2;
			cg_rjd2.bodyA = b;
			cg_rjd2.bodyB = conn_gear2;
			cg_rjd2.localAnchorA = b->GetLocalPoint(conn_gear2->GetPosition());
			cg_rjd2.localAnchorB = conn_gear2->GetLocalPoint(conn_gear2->GetPosition());
			m_world->CreateJoint(&cg_rjd2);

			b2Body *conn_gear3 = createGear(3.2, GEAR_LAYER6, 18.4, 20);
			b2RevoluteJointDef cg_rjd3;
			cg_rjd3.bodyA = b;
			cg_rjd3.bodyB = conn_gear3;
			cg_rjd3.localAnchorA = b->GetLocalPoint(conn_gear3->GetPosition());
			cg_rjd3.localAnchorB = conn_gear3->GetLocalPoint(conn_gear3->GetPosition());
			m_world->CreateJoint(&cg_rjd3);

			b2Body *conn_gear4 = createGear(3.2, GEAR_LAYER6, 24.6, 20);
			b2RevoluteJointDef cg_rjd4;
			cg_rjd4.bodyA = b;
			cg_rjd4.bodyB = conn_gear4;
			cg_rjd4.localAnchorA = b->GetLocalPoint(conn_gear4->GetPosition());
			cg_rjd4.localAnchorB = conn_gear4->GetLocalPoint(conn_gear4->GetPosition());
			m_world->CreateJoint(&cg_rjd4);

			b2PolygonShape rod_sm;
			rod_sm.SetAsBox(3.4, 0.1, b2Vec2(3, 0), 0);
			b2FixtureDef rod_fdm;
			rod_fdm.shape = &rod_sm;
			rod_fdm.density = 1.0;
			rod_fdm.filter.categoryBits = NC_LAYER;
			rod_fdm.filter.maskBits = 0x0000;
			conn_gear4->CreateFixture(&rod_fdm);

			b2CircleShape moon_shape;
			moon_shape.m_radius = 1;
			b2BodyDef moon_bd;
			moon_bd.type = b2_dynamicBody;
			moon_bd.position.Set(30, 20);
			b2Body *moon = m_world->CreateBody(&moon_bd);
			b2FixtureDef moon_body_fd;
			moon_body_fd.filter.categoryBits = NC_LAYER;
			moon_body_fd.filter.maskBits = 0x0000;
			moon_body_fd.shape = &moon_shape;
			moon_body_fd.density = 0.0;
			moon->CreateFixture(&moon_body_fd);
			b2RevoluteJointDef moonjoint;
			moonjoint.bodyA = conn_gear4;
			moonjoint.bodyB = moon;
			moonjoint.localAnchorA = conn_gear4->GetLocalPoint(moon->GetPosition());
			moonjoint.localAnchorB = moon->GetLocalPoint(moon->GetPosition());
			m_world->CreateJoint(&moonjoint);

			b2CircleShape earth_shape;
			earth_shape.m_radius = 4;
			b2BodyDef earth_bd;
			earth_bd.type = b2_dynamicBody;
			earth_bd.position.Set(24.6, 20);
			b2Body *earth = m_world->CreateBody(&earth_bd);
			b2FixtureDef earth_body_fd;
			earth_body_fd.filter.categoryBits = NC_LAYER;
			earth_body_fd.filter.maskBits = 0x0000;
			earth_body_fd.shape = &earth_shape;
			earth_body_fd.density = 0.0;
			earth->CreateFixture(&earth_body_fd);
			b2RevoluteJointDef earthjoint;
			earthjoint.bodyA = conn_gear4;
			earthjoint.bodyB = earth;
			earthjoint.localAnchorA = conn_gear4->GetLocalPoint(earth->GetPosition());
			earthjoint.localAnchorB = earth->GetLocalPoint(earth->GetPosition());
			m_world->CreateJoint(&earthjoint);
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
		//! Venus conn gear 1
		{
			b2Body *b = createGear(5.2, GEAR_LAYER2, -15, 29);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
		}

		//! Venus conn gear 2
		{
			b2Body *b = createGear(2.0, GEAR_LAYER2, -8, 29);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
		}

		//! Venus conn gear 3
		{
			b2Body *b = createGear(4.95, GEAR_LAYER2, -1.3, 29);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
		}


		//! Venus
		{
			b2Body *b = createGear(4.5, GEAR_LAYER2, 0, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			b2PolygonShape rod_s;
			rod_s.SetAsBox(10.4, 0.2, b2Vec2(10, 0), 0);
			b2FixtureDef rod_fd;
			rod_fd.shape = &rod_s;
			rod_fd.density = 1.0f;
			rod_fd.filter.categoryBits = NC_LAYER;
			rod_fd.filter.maskBits = 0x0000;
			b->CreateFixture(&rod_fd);

			b2CircleShape planet_shape;
			planet_shape.m_radius = 3.5;
			b2BodyDef planet_bd;
			planet_bd.type = b2_dynamicBody;
			planet_bd.position.Set(20, 20);
			b2Body *planet = m_world->CreateBody(&planet_bd);
			b2FixtureDef planet_body_fd;
			planet_body_fd.filter.categoryBits = NC_LAYER;
			planet_body_fd.filter.maskBits = 0x0000;
			planet_body_fd.shape = &planet_shape;
			planet_body_fd.density = 0.0;
			planet->CreateFixture(&planet_body_fd);
			b2RevoluteJointDef planetjoint;
			planetjoint.bodyA = b;
			planetjoint.bodyB = planet;
			planetjoint.localAnchorA = b->GetLocalPoint(planet->GetPosition());
			planetjoint.localAnchorB = planet->GetLocalPoint(planet->GetPosition());
			m_world->CreateJoint(&planetjoint);
		}
		//! Jupiter conn gear 1
		{
			b2Body *b = createGear(1.2, GEAR_LAYER3, -9, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
		}
		//! Jupiter
		
		{
			b2Body *b = createGear(8, GEAR_LAYER3, 0, 20);
			b2RevoluteJointDef g1_jd;
			g1_jd.bodyA = b;

			g1_jd.bodyB = ground;

			g1_jd.localAnchorA = b->GetLocalPoint(b->GetPosition());
			g1_jd.localAnchorB = ground->GetLocalPoint(b->GetPosition());
			b2RevoluteJoint* dgs_joint = (b2RevoluteJoint*)m_world->CreateJoint(&g1_jd);
			b2PolygonShape rod_s;
			rod_s.SetAsBox(20.4, 0.2, b2Vec2(20, 0), 0);
			b2FixtureDef rod_fd;
			rod_fd.shape = &rod_s;
			rod_fd.density = 1.0f;
			rod_fd.filter.categoryBits = NC_LAYER;
			rod_fd.filter.maskBits = 0x0040;
			b->CreateFixture(&rod_fd);
			
			b2CircleShape planet_shape;
			planet_shape.m_radius = 4.5;
			b2BodyDef planet_bd;
			planet_bd.type = b2_dynamicBody;
			planet_bd.position.Set(40, 20);
			b2Body *planet = m_world->CreateBody(&planet_bd);
			b2FixtureDef planet_body_fd;
			planet_body_fd.filter.categoryBits = NC_LAYER;
			planet_body_fd.filter.maskBits = 0x0000;
			planet_body_fd.shape = &planet_shape;
			planet_body_fd.density = 0.0;
			planet->CreateFixture(&planet_body_fd);
			b2RevoluteJointDef planetjoint;
			planetjoint.bodyA = b;
			planetjoint.bodyB = planet;
			planetjoint.localAnchorA = b->GetLocalPoint(planet->GetPosition());
			planetjoint.localAnchorB = planet->GetLocalPoint(planet->GetPosition());
			m_world->CreateJoint(&planetjoint);
			
		}*/
  }

	
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
