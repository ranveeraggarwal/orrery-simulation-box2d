
/**
 * \brief Simulation of orrery
 * 
 * @file dominos.cpp
 * Description: Modified file from originally supplied cs296_base_code
 * supplied. The new file simulates a mechanical planetarium called an 
 * orrery. It basically has a lot of gears coupling the various planets
 * and they rotate about the center.
 */

#include "cs296_base.hpp"
#include "render.hpp"
#define UNUSED(x) (void)(x)
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




/**
   The function creates a toothed gear at a given position of a given 
   radius.
   
   @param[in]     r float value denoting the radius of desired gear
   @param[in]     layer The layer bit desired for the gear
   @param[in]     x The x coordinate for the gear
   @param[in]	  y the y coordinate for the gear
   @param[in]	  teeth If teeth are desired or not. Default: true
   @param[in]     density The density of the gear. Default: 1.0f
   @return Pointer b2Body* to the created gear
 */

	b2Body* dominos_t::createGear(float r, uint16 layer, float x, float y, bool teeth = true, float density = 1.0f)
	{
		//Radius adjusted to closest value so that the number of teeth
		//is an integer
		r = r;
		int n = int(5 * r);
		r = n/5.0 ;
		r = r - 0.3;
		//Gear body
		b2CircleShape gear_shape;
		gear_shape.m_radius = r;
		b2BodyDef gear_bd;
		gear_bd.type = b2_dynamicBody;
		gear_bd.position.Set(x, y);
		b2Body *gear_b = m_world->CreateBody(&gear_bd);
		b2FixtureDef gear_body_fd;
		//Setting mask and category for colission filtering
		gear_body_fd.filter.categoryBits = layer;
		gear_body_fd.filter.maskBits = layer;
		gear_body_fd.shape = &gear_shape;
		gear_body_fd.density = density;
		//For toothless gear
		if (!teeth) gear_body_fd.friction = 10000;
		gear_b->CreateFixture(&gear_body_fd);
		//Code for generating teeth
		if(teeth)
		for(int i=0; i<2*n; i++)
		{

			float pi = 3.151592654;
			
			// Shape of the gear teeth
			b2PolygonShape gear_tooth;
			////////////////////////////////////////////////////////////
			//Uncomment to use square gear teeth
			/*gear_tooth.SetAsBox(0.2, 0.1, 
					b2Vec2((r+0.2) * cos((pi*i)/n), 
					(r+0.2)* sin((pi*i)/n)) , (pi*i) / n);*/
			////////////////////////////////////////////////////////////		
					
			
			////////////////////////////////////////////////////////////
			//Uncomment to use pentagon teeth.
			//Calculating the position of teeth
			float xc = (r+0.2);
			float yc = 0;
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
		This function takes two bodies and connects them with a revolute
		joint with local anchor at the first body.
   

		@param[in]   b1 Pointer to the first body
		@param[in]   b2 Pointer to the second body
		@param[in]   motor_speed speed of motor. Default: 0
		@param[in]	 motor_torque max torque of motor. Default: 10
		@return Pointer b2RevoluteJoint* to the joint
	*/
	b2RevoluteJoint* dominos_t::fixCenterRevolute(b2Body *b1, b2Body *b2, float motor_speed = 0, float motor_torque = 100)
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
	/**
		This function takes two bodies and welds them together
   

		@param[in]   b1 Pointer to the first body
		@param[in]   b2 Pointer to the second body
		@return Pointer b2RevoluteJoint* to the joint
	*/
	b2WeldJoint* dominos_t::weld(b2Body *b1, b2Body *b2)
	{
		b2WeldJointDef wjd;
		wjd.Initialize(b1, b2, b1->GetPosition());
		return (b2WeldJoint*)m_world->CreateJoint(&wjd);
	}
	/**
		This function takes in a length and a radius and creates a
		planet of that radius at distance length from the center.
		@param[in]  length half of distance of planet from center
		@param[in]   radius radius of planet
		@param[in]   x To shift center. Default: 0 = no shift
		@return Pointer b2Body* to the created body
	*/
	b2Body* dominos_t::createPlanet(float length, float radius, float x = 0)
	{
		
		b2PolygonShape rod;
		rod.SetAsBox(length, 0.1);
		b2FixtureDef rodfixture;
		rodfixture.shape = &rod;
		rodfixture.density = 1;
		rodfixture.filter.categoryBits = 0x0800;
		rodfixture.filter.maskBits = 0x0000;
		b2BodyDef rodbd;
		rodbd.position.Set(length+x,0);
		rodbd.type = b2_dynamicBody;
		b2Body* rodb = m_world->CreateBody(&rodbd);
		rodb->CreateFixture(&rodfixture);
		
		b2CircleShape planet;
		planet.m_radius = radius;
		b2FixtureDef planetfixture;
		planetfixture.shape = &planet;
		planetfixture.density = 1;
		planetfixture.filter.categoryBits = 0x0800;
		planetfixture.filter.maskBits = 0x0000;
		b2BodyDef planetbd;
		planetbd.type = b2_dynamicBody;
		planetbd.position.Set(2*length+x,0);
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


    ///Ground
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
			shape.Set(b2Vec2(0.01, 0.0), b2Vec2(-0.01, 0.0));
			ground->CreateFixture(&shape, 0.0);
		}
    
		//!Layer bits
		uint16 DRIVER_GEAR_LAYER = 0x0001, MARS_GEAR_LAYER = 0x0002, EARTH_GEAR_LAYER = 0x0004, MOON1_GEAR_LAYER = 0x0008,
				MOON2_GEAR_LAYER = 0x0010, VENUS_GEAR_LAYER = 0x0020, MERCURY_GEAR_LAYER = 0x0040;
		//!Planets layer
		//!Mercury
		b2Body* mercury_body = createPlanet(9, 1);
		
		//!Venus
		b2Body* venus_body = createPlanet(12, 1.5);
		
		//!Moon
		b2Body* moon_body = createPlanet(2, 0.5, 33);
		
		//!Earth
		b2Body* earth_body = createPlanet(16.5, 3);

		//!Mars
		b2Body* mars_body = createPlanet(23, 2.5);
		
		
		//!Mercury gear layer
		//!Main gear for mercury planet
		b2Body* mercury_gear_body = createGear(3.5, MERCURY_GEAR_LAYER, 0, 0);
		//!Top layer of the venus->mercury coupling gear
		b2Body* merc_venus_gear2 = createGear(4, MERCURY_GEAR_LAYER,-5.13,5.13);
		
		//!Venus gear layer
		//!Main gear for venus planet
		b2Body* venus_gear_body = createGear(4.5, VENUS_GEAR_LAYER, 0, 0);
		//!Top layer of earth->venus coupling gear
		b2Body* venus_earth_gear2 = createGear(4.5, VENUS_GEAR_LAYER,-7.39,-4.5);
		//!Bottom layer of venus->mercury coupling gear
		b2Body* merc_venus_gear1 = createGear(3, VENUS_GEAR_LAYER,-5.13,5.13);
		
		//!Moon movement gear layer
		//!Gear on earth's center for the moon
		b2Body* moon3_gear_body = createGear(1.8,MOON2_GEAR_LAYER, 33, 0, true, 1);
		//!Gears coupling the central moon gear to the earth moon gear
		b2RevoluteJoint* moon_coupler_joints[7];
		UNUSED(moon_coupler_joints);
		b2Body* moon_couplers[7];
		for(int i = 0; i< 7; i++)
		{
			moon_couplers[i] = createGear(2, MOON2_GEAR_LAYER, 3.9*i+5.85, 0, true, 1);
			moon_coupler_joints[i]=fixCenterRevolute(moon_couplers[i], earth_body, 0, 0);
		}
		//!Central upper moon gear
		b2Body* moon2_gear_body = createGear(4,MOON2_GEAR_LAYER, 0, 0);
		
		//!Moon driving layer
		//!Central lower moon gear
		b2Body* moon1_gear_body = createGear(3,MOON1_GEAR_LAYER, 0, 0);
		//!Top layer of earth moon coupling to accelerate moon
		b2Body* earth_moon_gear2 = createGear(5.1 , MOON1_GEAR_LAYER,2.702,-7.423);
		
		//!Earth layer
		//!Upper layer of earth->mars coupling
		b2Body* earth_mars_gear2 = createGear(4, EARTH_GEAR_LAYER,3.74,8.02);
		//!Lower layer of earth->venus coupling
		b2Body* venus_earth_gear1 = createGear(3.9, EARTH_GEAR_LAYER,-7.39,-4.5);
		//!Main earth gear
		b2Body* earth_gear_body = createGear(5, EARTH_GEAR_LAYER, 0, 0);
		//!Bottom layer of earth moon coupling to accelerate moon
		b2Body* earth_moon_gear1 = createGear(3, EARTH_GEAR_LAYER,2.702,-7.423);
		
		//!Mars layer
		//!Upper layer of driver gear->mars coupling
		b2Body* mars_driver_gear2 = createGear(5, MARS_GEAR_LAYER,10.85,0);
		//!Lower layer of mars->earth coupling
		b2Body* earth_mars_gear1 = createGear(3, MARS_GEAR_LAYER,3.74,8.02);
		//!Main mars gear
		b2Body* mars_gear_body = createGear(6, MARS_GEAR_LAYER, 0, 0);
		
		//!Driver layer
		//!Bottom layer of driver->mars coupling
		b2Body* mars_driver_gear1 = createGear(4, DRIVER_GEAR_LAYER,10.85,0);
		//!Main driver gear
		b2Body* dri_gear_body = createGear(7, DRIVER_GEAR_LAYER, 0, 0);
		
		
		//!Joints, welds
		
		//!Mercury main gear revolute joint
		b2RevoluteJoint* mercury_gear_rev = fixCenterRevolute(mercury_gear_body, ground);
		UNUSED(mercury_gear_rev);
		//!Mercury planet weld to main gear
		b2WeldJoint* merc_main_weld = weld(mercury_gear_body,mercury_body);
		UNUSED(merc_main_weld);
		
		//!Driver mars coupling gears revolute joints and internal weld
		b2RevoluteJoint* mdg2_rev = fixCenterRevolute(mars_driver_gear2, ground);
		UNUSED(mdg2_rev);
		b2RevoluteJoint* mdg1_rev = fixCenterRevolute(mars_driver_gear1, ground);
		UNUSED(mdg1_rev);
		b2WeldJoint* mdg_12_weld = weld(mars_driver_gear1, mars_driver_gear2);
		UNUSED(mdg_12_weld);
		
		
		//!Moon gears joints
		b2RevoluteJoint* moon2_gear_rev = fixCenterRevolute(moon2_gear_body, ground);
		UNUSED(moon2_gear_rev);
		b2RevoluteJoint* moon1_gear_rev_joint = fixCenterRevolute(moon1_gear_body, ground);
		UNUSED(moon1_gear_rev_joint);
		b2WeldJoint* moon_dri_weld = weld(moon1_gear_body, moon2_gear_body);
		UNUSED(moon_dri_weld);
		
		//!Mars earth coupling gears rev joints and welds
		b2RevoluteJoint* emg2_rev_joint = fixCenterRevolute(earth_mars_gear2, ground);
		UNUSED(emg2_rev_joint);
		b2RevoluteJoint* emg1_rev_joint = fixCenterRevolute(earth_mars_gear1, ground);
		UNUSED(emg1_rev_joint);
		b2WeldJoint* emg_12_weld_joint = weld(earth_mars_gear1, earth_mars_gear2);
		UNUSED(emg_12_weld_joint);
		
		//!Earth moon acceleration gear rev joints and welds
		b2RevoluteJoint* emoong2_rev_joint = fixCenterRevolute(earth_moon_gear2, ground);
		UNUSED(emoong2_rev_joint );
		b2RevoluteJoint* emoong1_rev_joint = fixCenterRevolute(earth_moon_gear1, ground);
		UNUSED(emoong1_rev_joint);
		b2WeldJoint* emoong_12_weld_joint = weld(earth_moon_gear1, earth_moon_gear2);
		UNUSED(emoong_12_weld_joint);
		//!Venus earth coupling gears rev joint and welds
		b2RevoluteJoint* veg2_rev_joint = fixCenterRevolute(venus_earth_gear2, ground);
		UNUSED(veg2_rev_joint);
		b2RevoluteJoint* veg1_rev_joint = fixCenterRevolute(venus_earth_gear1, ground);
		UNUSED(veg1_rev_joint);
		b2WeldJoint* veg_12_weld_joint = weld(venus_earth_gear1, venus_earth_gear2);
		UNUSED(veg_12_weld_joint);
		
		//!Main venus gear joints
		b2RevoluteJoint* venus_gear_rev_joint = fixCenterRevolute(venus_gear_body, ground);
		UNUSED(venus_gear_rev_joint );
		weld(venus_gear_body,venus_body);
		
		//!Venus mercury coupling rev joints and welds
		b2RevoluteJoint* mvg2_rev_joint = fixCenterRevolute(merc_venus_gear2, ground);
		UNUSED(mvg2_rev_joint);
		b2RevoluteJoint* mvg1_rev_joint = fixCenterRevolute(merc_venus_gear1, ground);
		UNUSED(mvg1_rev_joint);
		b2WeldJoint* mvg_12_weld_joint = weld(merc_venus_gear1, merc_venus_gear2);
		UNUSED(mvg_12_weld_joint);
		
		//!Main earth gear joints
		b2RevoluteJoint* earth_gear_rev_joint = fixCenterRevolute(earth_gear_body, ground);
		UNUSED(earth_gear_rev_joint);
		weld(earth_gear_body,earth_body);
		
		//!Main moon gear on earth joints
		b2RevoluteJoint* moon3_gear_rev_joint = fixCenterRevolute(moon3_gear_body, earth_gear_body);
		UNUSED( moon3_gear_rev_joint);
		weld(moon3_gear_body, moon_body);
		
		//!Main mars gear
		b2RevoluteJoint* mars_gear_rev_joint = fixCenterRevolute(mars_gear_body, ground);
		UNUSED(mars_gear_rev_joint);
		weld(mars_gear_body,mars_body);

		//!Main driver gear
		b2RevoluteJoint* dri_gear_rev_joint = fixCenterRevolute(dri_gear_body, ground, 0.5, 100000);
		UNUSED(dri_gear_rev_joint);
		
		
  }

	
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
